/*
  stepper.c - 步进电机驱动程序：使用步进电机执行运动计划
  Grbl的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以在自由软件基金会发布的 GNU 通用公共许可证条款下
  重新分发和/或修改它，可以使用许可证的第 3 版，或（如果您愿意）任何更新的版本。

  Grbl 的发布目的是为了能起到一定的作用，
  但它不提供任何保证；甚至没有关于
  适销性或适用于特定用途的隐含保证。有关详细信息，请参见
  GNU 通用公共许可证。

  您应该已随 Grbl 一起收到了 GNU 通用公共许可证的副本。
  如果没有，请访问 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 一些有用的常量。
#define DT_SEGMENT (1.0 / (ACCELERATION_TICKS_PER_SECOND * 60.0)) // 分钟/段
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)

// 定义自适应多轴步进平滑（AMASS）级别和截止频率。最高级别
// 频率区间从 0Hz 开始并结束于其截止频率。下一低级别频率区间
// 从下一个更高的截止频率开始，以此类推。每一级别的截止频率必须
// 仔细考虑如何过度驱动步进 ISR、16 位定时器的准确性以及 CPU 开销。级别 0（无 AMASS，正常操作）
// 频率区间从级别 1 的截止频率开始，并一直到 CPU 允许的最快速度（在有限测试中超过 30kHz）。
// 注意：AMASS 截止频率乘以 ISR 过度驱动因子不得超过最大步进频率。
// 注意：当前设置使 ISR 过度驱动到不超过 16kHz，平衡了 CPU 开销
// 和定时器精度。不要更改这些设置，除非您清楚您在做什么。
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0：正常操作。无 AMASS。没有上限截止频率。从 LEVEL1 截止频率开始。
#define AMASS_LEVEL1 (F_CPU / 8000) // 过度驱动 ISR（x2）。定义为 F_CPU/（以 Hz 为单位的截止频率）
#define AMASS_LEVEL2 (F_CPU / 4000) // 过度驱动 ISR（x4）
#define AMASS_LEVEL3 (F_CPU / 2000) // 过度驱动 ISR（x8）

// 存储用于段缓冲区中段的规划块 Bresenham 算法执行数据。通常，该缓冲区是部分使用的，但在最坏情况下，它不会超过可访问的步进缓冲区段数（SEGMENT_BUFFER_SIZE-1）。
// 注意：此数据是从预处理的规划块中复制的，以便规划块在被段缓冲区完全使用和完成时可以被丢弃。同时，AMASS 会修改此数据以便自用。
typedef struct
{
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
  uint8_t is_pwm_rate_adjusted; // 跟踪需要恒定激光功率/速率的运动
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE - 1];

// 主步进段环形缓冲区。包含供步进算法执行的小而短的直线段，这些段从规划缓冲区的第一个块中按需“取出”。
// 一旦“取出”，段缓冲区中的步数就无法被规划器修改，而规划块中的其余步数仍然可以。
typedef struct
{
  uint16_t n_step;          // 此段要执行的步事件数
  uint16_t cycles_per_tick; // 每个 ISR tick 的步进距离，即步进速率。
  uint8_t st_block_index;   // 步进块数据索引。使用此信息执行该段。
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  uint8_t amass_level; // 指示 ISR 执行该段的 AMASS 级别
#else
  uint8_t prescaler; // 没有 AMASS 时，需要一个预分频器来调整慢速计时。
#endif
  uint16_t spindle_pwm;
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// 步进 ISR 数据结构。包含主步进 ISR 的运行数据。
typedef struct
{
  // 用于 Bresenham 直线算法
  uint32_t counter_x, // Bresenham 线跟踪器的计数器变量
      counter_y,
      counter_z
#ifdef A_AXIS
      ,
      counter_a
#endif
#ifdef B_AXIS
      ,
      counter_b
#endif
#ifdef C_AXIS
      ,
      counter_c
#endif
#ifdef D_AXIS
      ,
      counter_d
#endif
      ;

#ifdef STEP_PULSE_DELAY
  uint8_t step_bits; // 存储 out_bits 输出以完成步进脉冲延迟
#endif

  uint8_t execute_step;    // 为每个中断标记步进执行。
  uint8_t step_pulse_time; // 步进脉冲上升后的复位时间
  uint8_t step_outbits;    // 要输出的下一个步进位
  uint8_t dir_outbits;
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  uint32_t steps[N_AXIS];
#endif

  uint16_t step_count;      // 线段运动中剩余的步数
  uint8_t exec_block_index; // 跟踪当前 st_block 索引。更改指示新块。
  st_block_t *exec_block;   // 指向正在执行段的块数据的指针
  segment_t *exec_segment;  // 指向正在执行段的指针
} stepper_t;
static stepper_t st;

// 步进段环形缓冲区索引
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// 步进和方向端口反转掩码。
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

// 用于避免“步进驱动中断”的 ISR 嵌套。虽然应该不会发生。
static volatile uint8_t busy;

// 从规划缓冲区准备步进段的指针。仅由主程序访问。指针可以计划提前执行的段或规划块。
static plan_block_t *pl_block;    // 指向准备的规划块的指针
static st_block_t *st_prep_block; // 指向准备的步进块数据的指针

// 段准备数据结构。包含基于当前执行的规划块计算新段所需的所有信息。
typedef struct
{
  uint8_t st_block_index; // 正在准备的步进通用数据块的索引
  uint8_t recalculate_flag;

  float dt_remainder;
  float steps_remaining;
  float step_per_mm;
  float req_mm_increment;

#ifdef PARKING_ENABLE
  uint8_t last_st_block_index;
  float last_steps_remaining;
  float last_step_per_mm;
  float last_dt_remainder;
#endif

  uint8_t ramp_type;      // 当前段的坡道状态
  float mm_complete;      // 当前规划块末尾的速度曲线结束（毫米）。
                          // 注意：此值在转换时必须与步进（无小数部分）一致。
  float current_speed;    // 段缓冲区末尾的当前速度（mm/min）
  float maximum_speed;    // 执行块的最大速度。不总是名义速度。（mm/min）
  float exit_speed;       // 执行块的退出速度（mm/min）
  float accelerate_until; // 加速坡道从块末端测量的结束位置（毫米）
  float decelerate_after; // 减速坡道从块末端测量的开始位置（毫米）

  float inv_rate; // 用于 PWM 激光模式加快段计算。
  uint16_t current_spindle_pwm;
} st_prep_t;
static st_prep_t prep;

/*    块速度曲线定义
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    d
    |               块 1              ^      块 2          |    ed
                                       |
                  时间 ----->      示例：块 2 的进入速度为最大交界速度

  规划块缓冲区是基于恒定加速度速度曲线进行规划的，并且在块交界处
  持续连接，如上所示。然而，规划器仅主动计算最佳速度规划的块进入速度，
  而不计算块内部的速度曲线。这些速度曲线是在步进算法执行时临时计算的，
  仅由 7 种可能的曲线类型组成：仅巡航、巡航-减速、加速-巡航、仅加速、
  仅减速、完全梯形和三角形（无巡航）。

                                        最大速度 (< 名义速度) ->  +
                    +--------+ <- 最大速度 (= 名义速度)          /|\
                   /          \                                           / | \
 当前速度 -> +            \                                         /  |  + <- 退出速度
              |             + <- 退出速度                         /   |  |
              +-------------+                     当前速度 -> +----+--+
               时间 -->  ^  ^                                           ^  ^
                         |  |                                           |  |
                减速后（以毫米为单位）                             减速后（以毫米为单位）
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                加速前（以毫米为单位）                             加速前（以毫米为单位）

  步进段缓冲区计算执行块的速度曲线，并跟踪步进算法准确追踪曲线所需的关键参数。
  这些关键参数在上面的插图中显示并定义。
*/

// 步进器状态初始化。仅当 st.cycle_start 标志启用时，周期才应开始。
// 启动初始化和限制调用此函数，但不应启动周期。
void st_wake_up()
{
  // 启用步进驱动器。
  if (bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE))
  {
    STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);
  }
  else
  {
    STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);
  }

  // 初始化步进输出位，以确保第一次 ISR 调用不会步进。
  st.step_outbits = step_port_invert_mask;

// 根据设置初始化步进脉冲定时。确保在重写后更新。
#ifdef STEP_PULSE_DELAY
  // 设置方向引脚设定后的总步进脉冲时间。通过示波器进行临时计算。
  st.step_pulse_time = -(((settings.pulse_microseconds + STEP_PULSE_DELAY - 2) * TICKS_PER_MICROSECOND) >> 3);
  // 设置方向引脚写入与步进命令之间的延迟。
  OCR0A = -(((settings.pulse_microseconds) * TICKS_PER_MICROSECOND) >> 3);
#else // 正常操作
  // 设置步进脉冲时间。通过示波器进行临时计算。使用二进制补码。
  st.step_pulse_time = -(((settings.pulse_microseconds - 2) * TICKS_PER_MICROSECOND) >> 3);
#endif

  // 启用步进驱动器中断
  TIMSK1 |= (1 << OCIE1A);
}

// 步进器关闭
void st_go_idle()
{
  // 禁用步进驱动器中断。如果正在运行，则允许步进端口重置中断完成。
  TIMSK1 &= ~(1 << OCIE1A);                                       // 禁用 Timer1 中断
  TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11))) | (1 << CS10); // 重置时钟至无分频。
  busy = false;

  // 设置步进驱动器空闲状态，禁用或启用，取决于设置和情况。
  bool pin_state = false; // 保持启用。
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING)
  {
    // 强制步进器停留，锁定轴在定义的时间内，以确保轴完全停止
    // 而不是因上次运动的残余惯性力漂移。
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // 强制。禁用步进器。
  }
  if (bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE))
  {
    pin_state = !pin_state;
  } // 应用引脚反转。
  if (pin_state)
  {
    STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);
  }
  else
  {
    STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);
  }
}

/* “步进驱动器中断” - 该定时器中断是 Grbl 的核心。Grbl 使用
   久经考验的 Bresenham 线算法来管理和精确同步多轴移动。
   与流行的 DDA 算法不同，Bresenham 算法不易受到数值
   舍入误差的影响，只需快速整数计数器，意味着低计算开销
   和最大化 Arduino 的能力。然而，Bresenham 算法的缺点是，对于某些多轴运动，
   非主导轴可能会出现不平滑的步进脉冲序列，或者别名，这可能导致奇怪的
   可听噪声或抖动。在低步进频率（0-5kHz）下，这一点尤为明显，可能会引起运动问题，
   但在更高频率下通常不是物理问题，尽管会发出声音。
     为了提高 Bresenham 的多轴性能，Grbl 使用我们称之为自适应多轴
   步进平滑（AMASS）算法，正如名称所暗示的那样。在较低的步进频率下，
   AMASS 人工增加 Bresenham 的分辨率而不影响算法的
   固有精确性。AMASS 根据要执行的步进频率自动调整其分辨率级别，
   意味着即使在更低的步进频率下，步进平滑级别也会增加。
   从算法上讲，AMASS 通过对每个 AMASS 级别的 Bresenham 步数进行简单的位移来实现。
   例如，对于级别 1 的步进平滑，我们对 Bresenham 步进事件计数进行位移，
   有效地将其乘以 2，同时轴的步数保持不变，然后将步进器 ISR 频率加倍。
   实际上，我们允许非主导的 Bresenham 轴在中间的 ISR 时钟中步进，
   而主导轴在每两个 ISR 时钟中步进，而不是在传统意义上的每个 ISR 时钟。
   在 AMASS 级别 2，我们再次进行位移，因此非主导的 Bresenham 轴可以在四个 ISR 时钟内步进，
   主导轴每四个 ISR 时钟步进，并将步进器 ISR 频率四倍化。依此类推。
   这在实际效果上几乎消除了 Bresenham 算法的多轴别名问题，并且
   不会显著改变 Grbl 的性能，实际上更高效地利用了所有配置中未使用的 CPU 周期。
     AMASS 通过要求始终执行完整的 Bresenham 步进来保持 Bresenham 算法的精确性，
   无论 AMASS 级别如何。这意味着对于 AMASS 级别 2，必须完成所有四个
   中间步进，以便始终保留基线 Bresenham（级别 0）计数。同样，
   AMASS 级别 3 意味着必须执行所有八个中间步进。
   尽管 AMASS 级别在现实中是任意的，其中基线 Bresenham 计数可以
   乘以任何整数值，但乘以 2 的幂只是为了减轻 CPU 开销，便于位移整数操作。
     该中断的设计简单且低效。所有计算重任，如
   确定加速度，都是在其他地方执行的。该中断从步进段缓冲区中弹出
   预计算的段，定义为在 n 个步进中的恒定速度，然后通过适当地脉冲步进引脚执行它们，
   使用 Bresenham 算法。该 ISR 得到了步进端口重置中断的支持，
   它在每次脉冲后用于重置步进端口。Bresenham 线追踪算法
   与这两个中断同时控制所有步进器输出。

   注意：该中断必须尽可能高效，并在下一个 ISR 时钟之前完成，
   对于 Grbl，必须少于 33.3 微秒（@30kHz ISR 频率）。示波器测量的 ISR 时间
   通常为 5 微秒，最大为 25 微秒，远低于要求。
   注意：该 ISR 期望每个段至少执行一次步进。
*/
// TODO: 以某种方式替换 ISR 中 int32 位置计数器的直接更新。也许使用更小的
// int8 变量，仅在段完成时更新位置计数器。这在探测和归位循环中可能变得复杂，
// 需要真实的实时位置。
ISR(TIMER1_COMPA_vect)
{
  if (busy)
  {
    return;
  } // 忙标志用于避免重新进入此中断

  // 在我们步进之前，设置方向引脚几纳秒
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

// 然后脉冲步进引脚
#ifdef STEP_PULSE_DELAY
  st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // 存储 out_bits 以防覆盖。
#else                                                        // 正常操作
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
#endif

  // 启用步进脉冲重置定时器，以便步进端口重置中断可以在
  // 精确的 settings.pulse_microseconds 微秒后重置信号，独立于主 Timer1 分频器。
  TCNT0 = st.step_pulse_time; // 重新加载 Timer0 计数器
  TCCR0B = (1 << CS01);       // 启动 Timer0。全速，1/8 分频

  busy = true;
  sei(); // 重新启用中断，以允许步进端口重置中断准时触发。
         // 注意：该 ISR 中的剩余代码将在返回到主程序之前完成。

  // 如果没有步进段，尝试从步进缓冲区中弹出一个
  if (st.exec_segment == NULL)
  {
    // 缓冲区中有内容吗？如果有，加载并初始化下一个步进段。
    if (segment_buffer_head != segment_buffer_tail)
    {
      // 初始化新的步进段并加载要执行的步数
      st.exec_segment = &segment_buffer[segment_buffer_tail];

#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // 当 AMASS 被禁用时，为步频较慢的段设置定时器分频器（< 250Hz）。
      TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (st.exec_segment->prescaler << CS10);
#endif

      // 初始化每步的步进段定时和加载要执行的步数。
      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; // 注意：当移动缓慢时，有时可能为零。
      // 如果新段开始一个新的规划块，初始化步进器变量和计数器。
      // 注意：当段数据索引变化时，表示一个新的规划块。
      if (st.exec_block_index != st.exec_segment->st_block_index)
      {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        // 初始化 Bresenham 线和距离计数器
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
#ifdef A_AXIS
        st.counter_a = st.counter_x;
#endif
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // 启用 AMASS 时，根据 AMASS 级别调整 Bresenham 轴增量计数器。
      st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
      st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
      st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
#ifdef A_AXIS
      st.steps[A_AXIS] = st.exec_block->steps[A_AXIS] >> st.exec_segment->amass_level;
#endif
#ifdef B_AXIS
      st.steps[B_AXIS] = st.exec_block->steps[B_AXIS] >> st.exec_segment->amass_level;
#endif
#ifdef C_AXIS
      st.steps[C_AXIS] = st.exec_block->steps[C_AXIS] >> st.exec_segment->amass_level;
#endif
#ifdef D_AXIS
      st.steps[D_AXIS] = st.exec_block->steps[D_AXIS] >> st.exec_segment->amass_level;
#endif
#endif

      // 在加载段时，设置实时主轴输出，正好在第一次步进之前。
      spindle_set_speed(st.exec_segment->spindle_pwm);
    }
    else
    {
      // 段缓冲区为空。关闭。
      st_go_idle();
      // 确保在速率控制运动完成时，PWM 设置正确。
      if (st.exec_block->is_pwm_rate_adjusted)
      {
        spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
      }
      system_set_exec_state_flag(EXEC_CYCLE_STOP); // 标记主程序为循环结束
      return;                                      // 没有什么可做的，退出。
    }
  }

  // 检查探测状态。
  if (sys_probe_state == PROBE_ACTIVE)
  {
    probe_state_monitor();
  }

  // 重置步进输出位。
  st.step_outbits = 0;

// 通过 Bresenham 线算法执行步进位移配置
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_x += st.steps[X_AXIS];
#else
  st.counter_x += st.exec_block->steps[X_AXIS];
#endif
  if (st.counter_x > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << X_DIRECTION_BIT))
    {
      sys_position[X_AXIS]--;
    }
    else
    {
      sys_position[X_AXIS]++;
    }
  }
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_y += st.steps[Y_AXIS];
#else
  st.counter_y += st.exec_block->steps[Y_AXIS];
#endif
  if (st.counter_y > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << Y_DIRECTION_BIT))
    {
      sys_position[Y_AXIS]--;
    }
    else
    {
      sys_position[Y_AXIS]++;
    }
  }
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_z += st.steps[Z_AXIS];
#else
  st.counter_z += st.exec_block->steps[Z_AXIS];
#endif
  if (st.counter_z > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << Z_DIRECTION_BIT))
    {
      sys_position[Z_AXIS]--;
    }
    else
    {
      sys_position[Z_AXIS]++;
    }
  }

#ifdef A_AXIS
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_a += st.steps[A_AXIS];
#else
  st.counter_a += st.exec_block->steps[A_AXIS];
#endif
  if (st.counter_a > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << A_STEP_BIT);
    st.counter_a -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << A_DIRECTION_BIT))
    {
      sys_position[A_AXIS]--;
    }
    else
    {
      sys_position[A_AXIS]++;
    }
  }
#endif
#ifdef B_AXIS
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_b += st.steps[B_AXIS];
#else
  st.counter_b += st.exec_block->steps[B_AXIS];
#endif
  if (st.counter_b > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << B_STEP_BIT);
    st.counter_b -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << B_DIRECTION_BIT))
    {
      sys_position[B_AXIS]--;
    }
    else
    {
      sys_position[B_AXIS]++;
    }
  }
#endif
#ifdef C_AXIS
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_c += st.steps[C_AXIS];
#else
  st.counter_c += st.exec_block->steps[C_AXIS];
#endif
  if (st.counter_c > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << C_STEP_BIT);
    st.counter_c -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << C_DIRECTION_BIT))
    {
      sys_position[C_AXIS]--;
    }
    else
    {
      sys_position[C_AXIS]++;
    }
  }
#endif

#ifdef D_AXIS
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  st.counter_d += st.steps[D_AXIS];
#else
  st.counter_d += st.exec_block->steps[D_AXIS];
#endif
  if (st.counter_d > st.exec_block->step_event_count)
  {
    st.step_outbits |= (1 << D_STEP_BIT);
    st.counter_d -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << D_DIRECTION_BIT))
    {
      sys_position[D_AXIS]--;
    }
    else
    {
      sys_position[D_AXIS]++;
    }
  }
#endif

  // 在归位周期中，锁定并防止所需轴移动。
  if (sys.state == STATE_HOMING)
  {
    st.step_outbits &= sys.homing_axis_lock;
  }

  st.step_count--; // 递减步事件计数
  if (st.step_count == 0)
  {
    // 段已完成。丢弃当前段并推进段索引。
    st.exec_segment = NULL;
    if (++segment_buffer_tail == SEGMENT_BUFFER_SIZE)
    {
      segment_buffer_tail = 0;
    }
  }

  st.step_outbits ^= step_port_invert_mask; // 应用步进端口反转掩码
  busy = false;
}

/* 步进端口重置中断：Timer0 OVF 中断处理步进脉冲的下降沿。
   这应该总是在下一个 Timer1 COMPA 中断之前触发，并且如果 Timer1 在完成移动后被禁用，则独立完成。
   注意：串行和步进中断之间的中断冲突可能会导致几微秒的延迟，如果它们在彼此之前执行。这不是大问题，但在高步进速率下，如果向 Grbl 添加另一个高频异步中断，则可能会导致问题。
*/

// 当 ISR_TIMER1_COMPAREA 设置电机端口位以执行一步时，会启用此中断。
// 此 ISR 在短时间后（settings.pulse_microseconds）重置电机端口，完成一步周期。
ISR(TIMER0_OVF_vect)
{
  // 重置步进引脚（保留方向引脚）
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK);
  TCCR0B = 0; // 禁用 Timer0，以防止在不需要时重新进入此中断。
}

#ifdef STEP_PULSE_DELAY
// 仅在启用 STEP_PULSE_DELAY 时使用此中断。这里，步进脉冲在 STEP_PULSE_DELAY 时间段结束后开始。
// 然后 ISR TIMER2_OVF 中断将在适当的 settings.pulse_microseconds 后触发，如正常操作。
// 方向、步进脉冲和步进完成事件之间的新时间安排在 st_wake_up() 例程中设置。
ISR(TIMER0_COMPA_vect)
{
  STEP_PORT = st.step_bits; // 开始步进脉冲。
}
#endif

// 生成用于步进中断驱动程序的步进和方向端口反转掩码。
void st_generate_step_dir_invert_masks()
{
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx = 0; idx < N_AXIS; idx++)
  {
    if (bit_istrue(settings.step_invert_mask, bit(idx)))
    {
      step_port_invert_mask |= get_step_pin_mask(idx);
    }
    if (bit_istrue(settings.dir_invert_mask, bit(idx)))
    {
      dir_port_invert_mask |= get_direction_pin_mask(idx);
    }
  }
}

// 重置和清除步进子系统变量
void st_reset()
{
  // 初始化步进驱动器空闲状态。
  st_go_idle();

  // 初始化步进算法变量。
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL; // 规划器块指针，用于段缓冲区
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // 为空 = 尾部
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; // 将方向位初始化为默认值。

  // 初始化步进和方向端口引脚。
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
}

// 初始化并启动步进电机子系统
void stepper_init()
{
  // 配置步进和方向接口引脚
  STEP_DDR |= STEP_MASK;
  STEPPERS_DISABLE_DDR |= 1 << STEPPERS_DISABLE_BIT;
  DIRECTION_DDR |= DIRECTION_MASK;

  // 配置 Timer 1：步进驱动中断
  TCCR1B &= ~(1 << WGM13); // 波形生成 = 0100 = CTC
  TCCR1B |= (1 << WGM12);
  TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0)); // 断开 OC1 输出
  // TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11))) | (1 << CS10); // 在 st_go_idle() 中设置。
  // TIMSK1 &= ~(1 << OCIE1A);  // 在 st_go_idle() 中设置。

  // 配置 Timer 0：步进端口重置中断
  TIMSK0 &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0)); // 断开 OC0 输出和 OVF 中断。
  TCCR0A = 0;                                                // 正常操作
  TCCR0B = 0;                                                // 在需要时禁用 Timer0
  TIMSK0 |= (1 << TOIE0);                                    // 启用 Timer0 溢出中断
#ifdef STEP_PULSE_DELAY
  TIMSK0 |= (1 << OCIE0A); // 启用 Timer0 比较匹配 A 中断
#endif
}

// 在执行块由新计划更新时由 planner_recalculate() 调用。
void st_update_plan_block_parameters()
{
  if (pl_block != NULL)
  { // 如果在新块开始时忽略。
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed * prep.current_speed; // 更新进入速度。
    pl_block = NULL;                                                     // 标记 st_prep_segment() 加载并检查活动速度轮廓。
  }
}

// 递增步段缓冲区块数据环形缓冲区。
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == (SEGMENT_BUFFER_SIZE - 1))
  {
    return (0);
  }
  return (block_index);
}

#ifdef PARKING_ENABLE
// 更改步段缓冲区的运行状态以执行特殊停车动作。
void st_parking_setup_buffer()
{
  // 如果必要，存储部分完成块的步进执行数据。
  if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK)
  {
    prep.last_st_block_index = prep.st_block_index;
    prep.last_steps_remaining = prep.steps_remaining;
    prep.last_dt_remainder = prep.dt_remainder;
    prep.last_step_per_mm = prep.step_per_mm;
  }
  // 设置标志以执行停车动作
  prep.recalculate_flag |= PREP_FLAG_PARKING;
  prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
  pl_block = NULL; // 始终重置停车动作以重新加载新块。
}

// 在停车动作后将步段缓冲区恢复到正常运行状态。
void st_parking_restore_buffer()
{
  // 如果必要，恢复部分完成块的步进执行数据和标志。
  if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK)
  {
    st_prep_block = &st_block_buffer[prep.last_st_block_index];
    prep.st_block_index = prep.last_st_block_index;
    prep.steps_remaining = prep.last_steps_remaining;
    prep.dt_remainder = prep.last_dt_remainder;
    prep.step_per_mm = prep.last_step_per_mm;
    prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
    prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm; // 重新计算该值。
  }
  else
  {
    prep.recalculate_flag = false;
  }
  pl_block = NULL; // 设置为重新加载下一个块。
}
#endif

/* 准备步段缓冲区。持续从主程序调用。

   段缓冲区是步进算法执行步骤与规划器生成的速度轮廓之间的中介缓冲区接口。
   步进算法仅在段缓冲区内执行步骤，并在从规划器缓冲区的第一个块“借出”步骤时由主程序填充。
   这使步进执行和规划优化过程原子化并相互保护。
   从规划器缓冲区“借出”的步数和段缓冲区中的段数经过尺寸和计算，使主程序中的任何操作所需时间不超过步进算法在重新填充之前清空它所需的时间。
   当前，段缓冲区保守地保持大约 40-50 毫秒的步数。
   注意：计算单位为步、毫米和分钟。
*/
void st_prep_buffer()
{
  // 在暂停状态下阻塞步进准备缓冲区，并且没有暂停运动要执行。
  if (bit_istrue(sys.step_control, STEP_CONTROL_END_MOTION))
  {
    return;
  }

  while (segment_buffer_tail != segment_next_head)
  { // 检查是否需要填充缓冲区。

    // 确定是否需要加载一个新的规划块，或者是否需要重新计算该块。
    if (pl_block == NULL)
    {
      // 查询规划器以获取排队的块
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION)
      {
        pl_block = plan_get_system_motion_block();
      }
      else
      {
        pl_block = plan_get_current_block();
      }
      if (pl_block == NULL)
      {
        return;
      } // 没有规划块。退出。

      // 检查是否需要仅重新计算速度曲线或加载新块。
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE)
      {

#ifdef PARKING_ENABLE
        if (prep.recalculate_flag & PREP_FLAG_PARKING)
        {
          prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
        }
        else
        {
          prep.recalculate_flag = false;
        }
#else
        prep.recalculate_flag = false;
#endif
      }
      else
      {

        // 加载该块的布雷森汉算法步进数据。
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        // 准备并复制来自新规划块的布雷森汉算法段数据，以便
        // 当段缓冲区完成规划块时，可以在段缓冲区完成准备块时丢弃它，但步进 ISR 仍在执行它。
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        uint8_t idx;
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        for (idx = 0; idx < N_AXIS; idx++)
        {
          st_prep_block->steps[idx] = pl_block->steps[idx];
        }
        st_prep_block->step_event_count = pl_block->step_event_count;
#else
        // 启用 AMASS 时，将所有布雷森汉数据简单地通过最大 AMASS
        // 水平进行位移乘法，这样我们在算法中的任何地方都不会超出原始数据进行除法。
        // 如果原始数据被除以，我们可能会因为整数舍入而丢失一个步进。
        for (idx = 0; idx < N_AXIS; idx++)
        {
          st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL;
        }
        st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
#endif

        // 初始化段缓冲区数据以生成段。
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining / pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;
        prep.dt_remainder = 0.0; // 为新的段块重置

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE))
        {
          // 新块在保持状态中加载。覆盖规划块的入口速度以强制减速。
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed * prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        }
        else
        {
          prep.current_speed = sqrt(pl_block->entry_speed_sqr);
        }

        // 设置激光模式变量。调整 PWM 速率的运动将始终在主轴关闭的情况下完成运动。
        st_prep_block->is_pwm_rate_adjusted = false;
        if (settings.flags & BITFLAG_LASER_MODE)
        {
          if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW)
          {
            // 预计算反向编程速率，以加快每个步进段的 PWM 更新。
            prep.inv_rate = 1.0 / pl_block->programmed_rate;
            st_prep_block->is_pwm_rate_adjusted = true;
          }
        }
      }

      /* ---------------------------------------------------------------------------------
       基于新的规划块的入口和出口速度计算速度曲线，或重新计算
       如果规划器更新了部分完成的规划块的曲线。对于强制减速的命令，例如从进给
       保持，覆盖规划速度并减速到目标出口速度。
      */
      prep.mm_complete = 0.0; // 默认情况下速度曲线在块末尾0.0mm处完成。
      float inv_2_accel = 0.5 / pl_block->acceleration;
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD)
      { // [强制减速至零速度]
        // 计算正在进行的进给保持的速度曲线参数。该曲线覆盖
        // 规划块曲线，强制减速至零速度。
        prep.ramp_type = RAMP_DECEL;
        // 计算相对于块末端的减速距离。
        float decel_dist = pl_block->millimeters - inv_2_accel * pl_block->entry_speed_sqr;
        if (decel_dist < 0.0)
        {
          // 在整个规划块中减速。进给保持的结束不在此块中。
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
        }
        else
        {
          prep.mm_complete = decel_dist; // 进给保持结束。
          prep.exit_speed = 0.0;
        }
      }
      else
      { // [正常操作]
        // 计算或重新计算准备好的规划块的速度曲线参数。
        prep.ramp_type = RAMP_ACCEL; // 初始化为加速坡道。
        prep.accelerate_until = pl_block->millimeters;

        float exit_speed_sqr;
        float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION)
        {
          prep.exit_speed = exit_speed_sqr = 0.0; // 强制在系统运动结束时停止。
        }
        else
        {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrt(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
        float nominal_speed_sqr = nominal_speed * nominal_speed;
        float intersect_distance =
            0.5 * (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr)
        { // 仅在覆盖减少时发生。
          prep.accelerate_until = pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0)
          { // 仅减速。
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // 计算覆盖块的出口速度，因为它与规划者出口速度不匹配。
            prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // 标记以加载下一个块作为减速覆盖。

            // TODO: 确定在仅减速时参数的正确处理。
            // 可能很棘手，因为入口速度将是当前速度，就像进给保持中一样。
            // 还要考虑与此相关的近零速度处理问题。
          }
          else
          {
            // 减速到巡航或巡航-减速类型。保证与更新计划相交。
            prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
            prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
        }
        else if (intersect_distance > 0.0)
        {
          if (intersect_distance < pl_block->millimeters)
          { // 可能是梯形或三角形类型
            // 注意：对于加速-巡航和仅巡航类型，以下计算将为0.0。
            prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance)
            { // 梯形类型
              prep.maximum_speed = nominal_speed;
              if (pl_block->entry_speed_sqr == nominal_speed_sqr)
              {
                // 巡航-减速或仅巡航类型。
                prep.ramp_type = RAMP_CRUISE;
              }
              else
              {
                // 完全梯形或加速-巡航类型
                prep.accelerate_until -= inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
              }
            }
            else
            { // 三角形类型
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0 * pl_block->acceleration * intersect_distance + exit_speed_sqr);
            }
          }
          else
          { // 仅减速类型
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
          }
        }
        else
        { // 仅加速类型
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }

      bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // 在更新块时强制更新。
    }

    // 初始化新段
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // 设置新段以指向当前段数据块。
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        通过确定在段时间 DT_SEGMENT 内行驶的总距离来计算此新段的平均速度。
      以下代码首先尝试根据当前坡道条件创建一个完整的段。如果在坡道状态变化时
      终止时段时间不完整，代码将继续循环遍历逐步的坡道状态以填充剩余段执行时间。
      但是，如果不完整的段在速度曲线的末尾终止，则该段被认为完成，
      尽管执行时间少于 DT_SEGMENT。
        速度曲线始终假定通过坡道序列进行：加速坡道、巡航状态和减速坡道。
      每个坡道的行驶距离可以从零到块的长度。速度曲线可以在
      规划块的末尾（典型）或在强制减速的中间（例如，从进给保持）结束。
    */
    float dt_max = DT_SEGMENT;                               // 最大段时间
    float dt = 0.0;                                          // 初始化段时间
    float time_var = dt_max;                                 // 时间工作变量
    float mm_var;                                            // mm-距离工作变量
    float speed_var;                                         // 速度工作变量
    float mm_remaining = pl_block->millimeters;              // 从块的末尾到新段的距离。
    float minimum_mm = mm_remaining - prep.req_mm_increment; // 确保至少有一步。
    if (minimum_mm < 0.0)
    {
      minimum_mm = 0.0;
    }

    do
    {
      switch (prep.ramp_type)
      {
      case RAMP_DECEL_OVERRIDE:
        speed_var = pl_block->acceleration * time_var;
        mm_var = time_var * (prep.current_speed - 0.5 * speed_var);
        mm_remaining -= mm_var;
        if ((mm_remaining < prep.accelerate_until) || (mm_var <= 0))
        {
          // 仅适用于减速覆盖的巡航或巡航-减速类型。
          mm_remaining = prep.accelerate_until; // 注意：在块末尾为 0.0
          time_var = 2.0 * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
          prep.ramp_type = RAMP_CRUISE;
          prep.current_speed = prep.maximum_speed;
        }
        else
        { // 中间减速覆盖坡道。
          prep.current_speed -= speed_var;
        }
        break;
      case RAMP_ACCEL:
        // 注意：加速坡道仅在第一次 do-while 循环中计算。
        speed_var = pl_block->acceleration * time_var;
        mm_remaining -= time_var * (prep.current_speed + 0.5 * speed_var);
        if (mm_remaining < prep.accelerate_until)
        { // 加速坡道结束。
          // 加速-巡航、加速-减速坡道交界处或块的末尾。
          mm_remaining = prep.accelerate_until; // 注意：在块末尾为 0.0
          time_var = 2.0 * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
          if (mm_remaining == prep.decelerate_after)
          {
            prep.ramp_type = RAMP_DECEL;
          }
          else
          {
            prep.ramp_type = RAMP_CRUISE;
          }
          prep.current_speed = prep.maximum_speed;
        }
        else
        { // 仅加速。
          prep.current_speed += speed_var;
        }
        break;
      case RAMP_CRUISE:
        // 注意：mm_var 用于保留未完成段的最后 mm_remaining，以便进行时间_var 计算。
        // 注意：如果 maximum_speed * time_var 值过低，舍入可能导致 mm_var 不变。为防止这种情况，
        //   在规划器中强制设置最小速度阈值。
        mm_var = mm_remaining - prep.maximum_speed * time_var;
        if (mm_var < prep.decelerate_after)
        { // 巡航结束。
          // 巡航-减速交界处或块的末尾。
          time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
          mm_remaining = prep.decelerate_after; // 注意：在块末尾为 0.0
          prep.ramp_type = RAMP_DECEL;
        }
        else
        { // 仅巡航。
          mm_remaining = mm_var;
        }
        break;
      default: // case RAMP_DECEL:
        // 注意：mm_var 作为杂项工作变量，以防在接近零速度时出错。
        speed_var = pl_block->acceleration * time_var; // 作为增量速度（mm/min）
        if (prep.current_speed > speed_var)
        { // 检查是否处于零速度或以下。
          // 计算从段末尾到块末尾的距离。
          mm_var = mm_remaining - time_var * (prep.current_speed - 0.5 * speed_var); // （mm）
          if (mm_var > prep.mm_complete)
          { // 典型情况。在减速坡道中。
            mm_remaining = mm_var;
            prep.current_speed -= speed_var;
            break; // 段完成。退出 switch-case 语句。继续 do-while 循环。
          }
        }
        // 否则，处于块末尾或强制减速的末尾。
        time_var = 2.0 * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
        mm_remaining = prep.mm_complete;
        prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // 将计算出的坡道时间添加到总段时间中。
      if (dt < dt_max)
      {
        time_var = dt_max - dt;
      } // **未完成** 在坡道交界处。
      else
      {
        if (mm_remaining > minimum_mm)
        { // 检查是否有非常慢的段落且步骤为零。
          // 增加段时间以确保在段中至少有一步。覆盖并循环
          // 直到达到 minimum_mm 或 mm_complete 的距离计算。
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        }
        else
        {
          break; // **完成** 退出循环。段执行时间已达到最大值。
        }
      }
    } while (mm_remaining > prep.mm_complete); // **完成** 退出循环。轮廓完成。

    /* -----------------------------------------------------------------------------------
      计算步进段的主轴速度 PWM 输出
    */

    if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM))
    {
      if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW))
      {
        float rpm = pl_block->spindle_speed;
        // 注意：进给和快速覆盖与 PWM 值无关，并且不会改变激光功率/速率。
        if (st_prep_block->is_pwm_rate_adjusted)
        {
          rpm *= (prep.current_speed * prep.inv_rate);
        }
        // 如果 current_speed 为零，则可能需要为 rpm_min * (100 / MAX_SPINDLE_SPEED_OVERRIDE)
        // 但这仅在运动期间是瞬时的。可能根本无关紧要。
        prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
      }
      else
      {
        sys.spindle_speed = 0.0;
        prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
      }
      bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
    }
    prep_segment->spindle_pwm = prep.current_spindle_pwm; // 重新加载段 PWM 值

    /* -----------------------------------------------------------------------------------
      计算段步率、待执行步骤，并应用必要的速率校正。
      注意：步骤是通过将剩余的毫米距离直接转换为标量计算的，而不是逐步统计每段执行的步骤。
      这有助于消除几个加法的浮点舍入问题。
      但是，由于浮点数只有 7.2 位有效数字，因此长时间以极高步数移动可能会超出浮点数的精度，
      这可能会导致步骤丢失。
      幸运的是，这种情况在 Grbl 支持的 CNC 机器中非常不太可能且不现实（即，以 200 步/mm 超过 10 米的轴移动）。
    */
    float step_dist_remaining = prep.step_per_mm * mm_remaining;       // 将 mm_remaining 转换为步骤
    float n_steps_remaining = ceil(step_dist_remaining);               // 向上取整当前剩余步骤
    float last_n_steps_remaining = ceil(prep.steps_remaining);         // 向上取整最后剩余步骤
    prep_segment->n_step = last_n_steps_remaining - n_steps_remaining; // 计算待执行的步骤数。

    // 如果我们处于进给保持的末尾而没有步骤可执行，则退出。
    if (prep_segment->n_step == 0)
    {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD)
      {
        // 减速到零速度不到一步，但已经非常接近。AMASS
        // 需要执行完整步骤。因此，只需退出。
        bit_true(sys.step_control, STEP_CONTROL_END_MOTION);
#ifdef PARKING_ENABLE
        if (!(prep.recalculate_flag & PREP_FLAG_PARKING))
        {
          prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK;
        }
#endif
        return; // 段未生成，但当前步骤数据仍保留。
      }
    }

    // 计算段步率。由于步骤是整数而毫米距离不是，
    // 每个段的末尾可能有不同数量的部分步骤未执行，因为步进 ISR 需要整体步骤以满足 AMASS 算法。为了补偿，我们跟踪执行前一个段的部分步骤所需的时间，并将其简单地应用于当前段的部分步骤，从而微调整体段速率以保持步骤输出准确。这些速率调整通常非常小，并不会对性能产生不利影响，但确保 Grbl 输出由规划器计算的确切加速度和速度曲线。
    dt += prep.dt_remainder;                                              // 应用前一个段部分步骤的执行时间
    float inv_rate = dt / (last_n_steps_remaining - step_dist_remaining); // 计算调整后的步骤速率反向

    // 计算预备段的每步 CPU 周期。
    uint32_t cycles = ceil((TICKS_PER_MICROSECOND * 1000000 * 60) * inv_rate); // （周期/步）

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                                                                               // 计算步进时序和多轴平滑级别。
    // 注意：AMASS 通过每个级别超驱动定时器，因此只需要一个预分频器。
    if (cycles < AMASS_LEVEL1)
    {
      prep_segment->amass_level = 0;
    }
    else
    {
      if (cycles < AMASS_LEVEL2)
      {
        prep_segment->amass_level = 1;
      }
      else if (cycles < AMASS_LEVEL3)
      {
        prep_segment->amass_level = 2;
      }
      else
      {
        prep_segment->amass_level = 3;
      }
      cycles >>= prep_segment->amass_level;
      prep_segment->n_step <<= prep_segment->amass_level;
    }
    if (cycles < (1UL << 16))
    {
      prep_segment->cycles_per_tick = cycles;
    } // < 65536 (16MHz 下 4.1ms)
    else
    {
      prep_segment->cycles_per_tick = 0xffff;
    } // 设定为可能的最低速度。
#else
                                                                               // 计算正常步进生成的步进时序和定时器预分频器。
    if (cycles < (1UL << 16))
    {                              // < 65536  (16MHz 下 4.1ms)
      prep_segment->prescaler = 1; // 预分频器：0
      prep_segment->cycles_per_tick = cycles;
    }
    else if (cycles < (1UL << 19))
    {                              // < 524288 (16MHz 下 32.8ms)
      prep_segment->prescaler = 2; // 预分频器：8
      prep_segment->cycles_per_tick = cycles >> 3;
    }
    else
    {
      prep_segment->prescaler = 3; // 预分频器：64
      if (cycles < (1UL << 22))
      { // < 4194304 (16MHz 下 262ms)
        prep_segment->cycles_per_tick = cycles >> 6;
      }
      else
      { // 设定为可能的最低速度。 （约 4 步/秒）。
        prep_segment->cycles_per_tick = 0xffff;
      }
    }
#endif

    // 段完成！增加段缓冲区索引，以便步进 ISR 可以立即执行它。
    segment_buffer_head = segment_next_head;
    if (++segment_next_head == SEGMENT_BUFFER_SIZE)
    {
      segment_next_head = 0;
    }

    // 更新适当的规划器和段数据。
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining) * inv_rate;

    // 检查退出条件并标记以加载下一个规划块。
    if (mm_remaining == prep.mm_complete)
    {
      // 规划块结束或强制终止。没有更多的距离可以执行。
      if (mm_remaining > 0.0)
      { // 在强制终止的末尾。
        // 重置准备参数以恢复，然后退出。允许步进 ISR 完成段队列，实时协议将在收到
        // ISR 的循环停止标志后设置新状态。Prep_segment 被阻止，直到那时。
        bit_true(sys.step_control, STEP_CONTROL_END_MOTION);
#ifdef PARKING_ENABLE
        if (!(prep.recalculate_flag & PREP_FLAG_PARKING))
        {
          prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK;
        }
#endif
        return; // 退出！
      }
      else
      { // 规划块结束
        // 规划块已完成。所有步骤已设置为在段缓冲区中执行。
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION)
        {
          bit_true(sys.step_control, STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // 设置指针以指示检查并加载下一个规划块。
        plan_discard_current_block();
      }
    }
  }
}

// 实时状态报告调用以获取当前执行的速度。此值
// 实际上不是当前速度，而是在段缓冲区中上一个步骤段中计算的速度。
// 它始终落后于最多段块数（-1）
// 除以加速度每秒的滴答数（ACCELERATION TICKS PER SECOND）秒数。
float st_get_realtime_rate()
{
  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR))
  {
    return prep.current_speed;
  }
  return 0.0f;
}