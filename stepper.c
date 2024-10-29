/*
  stepper.c - 步进电机驱动：使用步进电机执行运动计划
  Grbl的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon, Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以根据 GNU 通用公共许可证的条款重新分发和/或修改
  它，该许可证由自由软件基金会发布，许可证的版本为第 3 版，或
  （根据您的选择）任何更高版本。

  Grbl 的发行目的是希望它对您有用，
  但不提供任何担保；甚至没有对适销性或特定目的适用性的暗示担保。有关更多详细信息，请参阅
  GNU 通用公共许可证。

  您应该已经收到了一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 一些有用的常量。
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // 每段的最小时间
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)

// 定义自适应多轴步进平滑（AMASS）级别和截止频率。最高级别的
// 频率区间从 0Hz 开始，到其截止频率结束。下一个较低级别的频率区间
// 从下一个较高的截止频率开始，依此类推。每个级别的截止频率必须
// 认真考虑，以防超出步进 ISR 的处理能力、16 位定时器的精度和 CPU 的开销。级别 0（无 AMASS，正常操作）的
// 频率区间从级别 1 的截止频率开始，快到 CPU 所允许的速度（在有限测试中超过 30kHz）。
// 注意：AMASS 截止频率乘以 ISR 超过因素不得超过最大步进频率。
// 注意：当前设置将 ISR 超过限制在不超过 16kHz，以平衡 CPU 开销和
// 定时器精度。除非您知道自己在做什么，否则请勿更改这些设置。
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0：正常操作，无 AMASS，无上限截止频率，从 LEVEL1 截止频率开始。
#define AMASS_LEVEL1 (F_CPU/8000) // 超过限制 ISR（x2）。定义为 F_CPU/(截止频率 Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // 超过限制 ISR（x4）
#define AMASS_LEVEL3 (F_CPU/2000) // 超过限制 ISR（x8）

// 存储计划块的 Bresenham 算法执行数据，用于段缓冲区中的段。
// 通常，这个缓冲区是部分使用的，但在最坏的情况下，它永远不会超过可访问的步进缓冲区段数（SEGMENT_BUFFER_SIZE-1）。
// 注意：此数据是从准备好的计划块中复制的，以便在完全消耗和完成后，可以丢弃计划块。
// 此外，AMASS 会为其自身使用修改此数据。
typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
  uint8_t is_pwm_rate_adjusted; // 跟踪需要恒定激光功率/速率的运动
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// 主要的步进段环形缓冲区。包含小的、短的线段供步进
// 算法执行，这些线段是从计划缓冲区的第一个块中逐步“借出”的。
// 一旦“借出”，段缓冲区中的步骤不能被计划者修改，而剩余的计划块步骤仍然可以。
typedef struct {
  uint16_t n_step;           // 此段要执行的步进事件数量
  uint16_t cycles_per_tick;  // 每个 ISR 时钟周期内的步进距离，即步进速率。
  uint8_t  st_block_index;   // 步进块数据索引。使用此信息来执行此段。
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // 表示 ISR 执行此段的 AMASS 级别
  #else
    uint8_t prescaler;      // 如果没有 AMASS，则需要一个预分频器来调整慢速定时。
  #endif
  uint16_t spindle_pwm;
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// 步进 ISR 数据结构。包含主步进 ISR 的运行数据。
typedef struct {
  // 用于 Bresenham 线算法
  uint32_t counter_x,        // Bresenham 线追踪器的计数变量
           counter_y,
           counter_z
#ifdef A_AXIS
          , counter_a
#endif
#ifdef B_AXIS
          , counter_b
#endif
#ifdef C_AXIS
          , counter_c
#endif
          ;

  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // 存储用于完成步进脉冲延迟的 out_bits 输出
  #endif

  uint8_t execute_step;     // 标志每个中断的步进执行。
  uint8_t step_pulse_time;  // 步进上升后的步进脉冲重置时间
  uint8_t step_outbits;         // 下一个要输出的步进位
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // 线段运动中剩余的步数
  uint8_t exec_block_index; // 跟踪当前的 st_block 索引。更改表示新块。
  st_block_t *exec_block;   // 指向正在执行的段的块数据的指针
  segment_t *exec_segment;  // 指向正在执行的段的指针
} stepper_t;
static stepper_t st;

// 步进段环形缓冲区索引
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// 步进和方向端口反转掩码。
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

// 用于避免“步进驱动器中断”的 ISR 嵌套。这种情况不应发生。
static volatile uint8_t busy;

// 用于从计划缓冲区准备步进段的指针。仅由主程序访问。指针可能指向
// 正在执行的计划段或计划块之前的段。
static plan_block_t *pl_block;     // 指向正在准备的计划块的指针
static st_block_t *st_prep_block;  // 指向正在准备的步进块数据的指针

// 段准备数据结构。包含所有必要的信息以根据当前执行的计划块计算新的段。
typedef struct {
  uint8_t st_block_index;  // 正在准备的步进公共数据块的索引
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

  uint8_t ramp_type;      // 当前段的斜率状态
  float mm_complete;      // 当前计划块结束时的速度轮廓终点（mm）。
                          // 注意：此值在转换时必须与步进对齐（无尾数）。
  float current_speed;    // 段缓冲区末尾的当前速度（mm/min）
  float maximum_speed;    // 正在执行的块的最大速度。并不总是标称速度。（mm/min）
  float exit_speed;       // 正在执行的块的出口速度（mm/min）
  float accelerate_until; // 从块末尾测量的加速斜率结束位置（mm）
  float decelerate_after; // 从块末尾测量的减速斜率开始位置（mm）

  float inv_rate;    // 用于 PWM 激光模式以加快段计算的速度。
  uint16_t current_spindle_pwm; 
} st_prep_t;
static st_prep_t prep;

/*    块速度轮廓定义
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               块 1            ^      块 2          |    d
                                       |
                  时间 ----->      示例：块 2 的入口速度为最大交界速度

  计划块缓冲区假设恒定加速度速度轮廓，并在块交界处连续连接，如上所示。然而，
  计划者仅积极计算块入口速度以实现最佳速度计划，但不计算块内部
  速度轮廓。这些速度轮廓在步进算法执行时即兴计算，并且仅由 7 种可能类型的轮廓组成：
  仅巡航、巡航-减速、加速-巡航、仅加速、仅减速、完整梯形和三角形（无巡航）。

                                        最大速度 (< 标称速度) ->  +
                    +--------+ <- 最大速度 (= 标称速度)          /|\
                   /          \                                           / | \
 当前速度 -> +            \                                         /  |  + <- 出口速度
                  |             + <- 出口速度                         /   |  |
                  +-------------+                     当前速度 -> +----+--+
                   时间 -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                减速后位置（mm）                             减速后位置（mm）
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                加速至位置（mm）                             加速至位置（mm）

  步进段缓冲区计算正在执行的块的速度轮廓，并跟踪步进算法准确描绘轮廓所需的关键参数。
  这些关键参数如上图所示并定义。
*/

// 步进状态初始化。仅当 st.cycle_start 标志启用时，循环才应开始。
// 启动初始化和限位调用此函数，但不应启动循环。

void st_wake_up()
{
  // 启用步进驱动器。
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }

  // 初始化步进输出位，以确保第一次 ISR 调用时不进行步进。
  st.step_outbits = step_port_invert_mask;

  // 根据设置初始化步进脉冲定时。确保在重写后更新。
  #ifdef STEP_PULSE_DELAY
    // 设置方向引脚设置后的总步进脉冲时间。通过示波器进行的经验计算。
    st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
    // 设置方向引脚写入与步进命令之间的延迟。
    OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
  #else // 正常操作
    // 设置步进脉冲时间。通过示波器进行的经验计算。使用补码表示。
    st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
  #endif

  // 启用步进驱动器中断
  TIMSK1 |= (1<<OCIE1A);
}


// 步进器关闭
void st_go_idle()
{
  // 禁用步进驱动器中断。如果激活，允许步进端口重置中断完成。
  TIMSK1 &= ~(1<<OCIE1A); // 禁用 Timer1 中断
  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // 将时钟重置为无预分频。
  busy = false;

  // 设置步进驱动器的闲置状态，根据设置和情况禁用或启用。
  bool pin_state = false; // 保持启用。
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    // 强制步进器停留以锁定轴，在定义的时间内确保轴完全停止，
    // 不会因最后一次运动的残余惯性而漂移。
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // 覆盖。禁用步进器。
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } // 应用引脚反转。
  if (pin_state) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
}


/* “步进驱动器中断” - 此定时器中断是 Grbl 的工作马车。Grbl 使用
   久负盛名的 Bresenham 线算法来管理和精确同步多轴运动。
   与流行的 DDA 算法不同，Bresenham 算法不易受到数值
   舍入误差的影响，仅需要快速的整数计数器，这意味着低计算开销
   并最大化 Arduino 的能力。然而，Bresenham 算法的缺点是，对于某些多轴运动，
   非主导轴可能会遭受不平滑的步进脉冲序列或混叠，这可能导致奇怪的可听噪声或抖动。
   这种情况在低步频（0-5kHz）下尤其明显，但在较高频率下通常不会造成物理问题，尽管会可听到。
     为了改善 Bresenham 多轴性能，Grbl 使用我们称之为自适应多轴
   步进平滑（AMASS）算法，顾名思义，AMASS 在较低步频下
   人为地增加 Bresenham 的分辨率而不影响算法的
   固有准确性。AMASS 自动根据即将执行的步频调整其分辨率级别，这意味着对于
   更低的步频，步进平滑级别会增加。在算法上，AMASS 通过简单的位移
   来实现 Bresenham 步进计数。例如，对于级别 1 的步进平滑，我们通过位移
   Bresenham 步进事件计数，有效地将其乘以 2，而轴步进计数保持不变，然后将
   步进 ISR 频率加倍。实际上，我们允许非主导的 Bresenham 轴在中间 ISR 时钟周期内
   步进，而主导轴则每两个 ISR 时钟周期步进，而不是在传统意义上每个 ISR 时钟周期步进。在 AMASS
   级别 2 时，我们再次位移，因此非主导的 Bresenham 轴可以在四个 ISR 时钟周期中的任何一个内步进，主导轴每四个 ISR 时钟周期步进，并将
   步进 ISR 频率四倍增加。依此类推。这实际上消除了 Bresenham 算法的多轴混叠
   问题，并且并没有显著改变 Grbl 的性能，实际上，在所有配置中更有效地利用了未使用的 CPU 周期。
     AMASS 通过要求它始终执行一个完整的
   Bresenham 步进来保持 Bresenham 算法的准确性，无论 AMASS 级别如何。这意味着对于 AMASS 级别 2，所有四个
   中间步骤都必须完成，以确保基线 Bresenham（级别 0）计数始终
   保留。类似地，AMASS 级别 3 意味着所有八个中间步骤都必须执行。
   虽然 AMASS 级别在现实中是任意的，但基线 Bresenham 计数可以
   乘以任何整数值，但乘以 2 的幂只是为了简化
   计算开销的位移整数运算。
     该中断设计简单且愚蠢。所有计算重负荷，如
   确定加速度，都在其他地方进行。该中断从步进段缓冲区弹出预计算的段，
   定义为在 n 个步骤上保持恒定速度，然后通过适当地脉冲步进引脚执行它们，
   使用 Bresenham 算法。此 ISR 由步进端口重置中断支持，
   它在每个脉冲后重置步进端口。Bresenham 线追踪算法
   同时控制所有步进输出，并通过这两个中断实现。

   注意：此中断必须尽可能高效，并在下一个 ISR 时钟周期之前完成，
   对于 Grbl，必须小于 33.3微秒（@30kHz ISR 速率）。示波器测得 ISR 中的时间
   通常为 5微秒，最大为 25微秒，远低于要求。
   注意：此 ISR 期望每个段至少执行一次步进。
*/
// TODO: 找到替代方案以直接更新 ISR 中的 int32 位置计数器。可以考虑使用较小的
// int8 变量，并仅在段完成时更新位置计数器。这可能会在探测和归位周期中变得复杂，
// 这些周期需要真实的实时位置。

void st_wake_up()
{
  // 启用步进驱动器。
  if (bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE)) {
    STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);
  } else {
    STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);
  }

  // 初始化步进输出位，以确保第一次中断服务例程（ISR）调用时不会步进。
  st.step_outbits = step_port_invert_mask;

  // 从设置中初始化步进脉冲时序。这里是为了在重写后确保更新。
  #ifdef STEP_PULSE_DELAY
    // 在方向引脚设置后，设定总步进脉冲时间。根据示波器的经验计算。
    st.step_pulse_time = -(((settings.pulse_microseconds + STEP_PULSE_DELAY - 2) * TICKS_PER_MICROSECOND) >> 3);
    // 设置方向引脚写入与步进命令之间的延迟。
    OCR0A = -(((settings.pulse_microseconds) * TICKS_PER_MICROSECOND) >> 3);
  #else // 正常操作
    // 设置步进脉冲时间。根据示波器的经验计算。使用二的补数。
    st.step_pulse_time = -(((settings.pulse_microseconds - 2) * TICKS_PER_MICROSECOND) >> 3);
  #endif

  // 启用步进驱动器中断
  TIMSK1 |= (1 << OCIE1A);
}

// 步进器关闭
void st_go_idle()
{
  // 禁用步进驱动器中断。如果步进端口重置中断正在活动，则允许其完成。
  TIMSK1 &= ~(1 << OCIE1A); // 禁用Timer1中断
  TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11))) | (1 << CS10); // 将时钟重置为无分频。
  busy = false;

  // 设置步进驱动器的闲置状态，根据设置和情况决定禁用或启用。
  bool pin_state = false; // 保持启用。
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    // 强制步进器停留，以在定义的时间内锁定轴，确保轴完全停止，不会因最后一次移动后的惯性残留力漂移。
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // 覆盖。禁用步进器。
  }
  if (bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE)) {
    pin_state = !pin_state; // 应用引脚反转。
  }
  if (pin_state) {
    STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);
  } else {
    STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);
  }
}

/* "步进驱动器中断" - 此定时器中断是Grbl的工作马。Grbl使用
   久负盛名的Bresenham线算法来管理和精确同步多轴移动。
   与流行的DDA算法不同，Bresenham算法不容易受到数值
   舍入误差的影响，只需快速的整数计数器，意味着低计算开销
   并最大化Arduino的能力。然而，Bresenham算法的缺点是，对于某些多轴运动，
   非主导轴可能会遭受不平滑的步进脉冲列，或别名，这可能导致奇怪的可听噪音或震动。
   这在低步频（0-5kHz）时尤为明显，但在高频时通常不是物理问题，尽管可听。
     为了改善Bresenham多轴性能，Grbl使用我们称之为自适应多轴
   步进平滑（AMASS）算法，顾名思义。在较低的步频下，
   AMASS人为地提高Bresenham的分辨率，而不影响算法的
   固有精确性。AMASS会根据要执行的步频自动调整其分辨率级别，
   这意味着对于更低的步频，步进平滑级别会增加。算法上，AMASS通过简单的位移
   每个AMASS级别的Bresenham步数来实现。例如，对于级别1的步进平滑，
   我们通过位移Bresenham步事件计数，实际上将其乘以2，而轴的步数保持不变，
   然后将步进ISR频率翻倍。实际上，我们允许非主导的Bresenham轴在
   中间的ISR滴答中步进，而主导轴每两个ISR滴答步进，而不是在传统意义上的
   每个ISR滴答。在AMASS级别2时，我们再次位移，因此非主导的Bresenham轴可以在
   任何四个ISR滴答内步进，主导轴每四个ISR滴答步进，并四倍
   步进ISR频率。依此类推。这实际上消除了Bresenham算法的多轴别名
   问题，并不会显著改变Grbl的性能，反而更有效地利用了所有配置中未使用的CPU周期。
     AMASS通过要求始终执行完整的
   Bresenham步，保持Bresenham算法的精确性。这意味着对于AMASS级别2，
   必须完成所有四个中间步骤，以保持基线Bresenham（级别0）计数。
   同样，AMASS级别3意味着必须执行所有八个中间步骤。
   尽管AMASS级别实际上是任意的，基线Bresenham计数可以乘以任何整数，
   但通过二的幂乘法来简化CPU开销与位移整数操作。
     此中断的设计是简单而低效的。所有计算的重担，例如
   确定加速度，都是在其他地方执行的。此中断从步进段缓冲区弹出预计算的段，
   定义为在n个步数内以恒定速度，然后通过适当的脉冲步进引脚来执行它们
   通过Bresenham算法。此ISR由步进端口重置中断支持，该中断在每个脉冲后重置步进端口。
   Bresenham线追踪算法同时控制所有步进输出，通过这两个中断。

   注意：此中断必须尽可能高效，并在下一个ISR滴答之前完成，
   对于Grbl，必须小于33.3微秒（@30kHz ISR速率）。示波器测量的ISR时间
   通常为5微秒，最大为25微秒，远低于要求。
   注意：此ISR期望每个段至少执行一个步进。
*/
// TODO: 以某种方式替换ISR中的int32位置计数器的直接更新。也许使用更小的
// int8变量，并在段完成时仅更新位置计数器。这在探测和归位周期中会变得复杂，因为它们需要真实的实时位置。

ISR(TIMER1_COMPA_vect)
{
  if (busy) { return; } // 忙标志用于避免重新进入此中断

  // 在我们步进步进器之前的几个纳秒设置方向引脚
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

  // 然后脉冲步进引脚
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // 存储输出位以防止覆盖。
  #else  // 正常操作
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
  #endif

  // 启用步进脉冲重置计时器，以便步进端口重置中断可以在
  // 精确的settings.pulse_microseconds微秒后重置信号，与主Timer1分频器无关。
  TCNT0 = st.step_pulse_time; // 重新加载Timer0计数器
  TCCR0B = (1 << CS01); // 开始Timer0。全速，1/8分频

  busy = true;
  sei(); // 重新启用中断，以便步进端口重置中断按时触发。
         // 注意：此ISR中的剩余代码将在返回主程序之前完成。

  // 如果没有步进段，尝试从步进缓冲区弹出一个
  if (st.exec_segment == NULL) {
    // 缓冲区中有内容吗？如果有，加载并初始化下一个步进段。
    if (segment_buffer_head != segment_buffer_tail) {
      // 初始化新步进段并加载要执行的步数
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // 如果AMASS禁用，为具有慢速步进频率的段设置计时器分频器（<250Hz）。
        TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (st.exec_segment->prescaler << CS10);
      #endif

      // 初始化步进段的每步定时并加载要执行的步数。
      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; // 注意：在缓慢移动时，有时可以为零。
      // 如果新段开始新的规划块，初始化步进器变量和计数器。
      // 注意：当段数据索引更改时，这表示一个新的规划块。
      if (st.exec_block_index != st.exec_segment->st_block_index) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        // 初始化Bresenham线和距离计数器
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
#ifdef A_AXIS
        st.counter_a = st.counter_x;
#endif
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // 启用AMASS时，根据AMASS级别调整Bresenham轴递增计数器。
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

      #endif

      // 设置实时主轴输出，因为段在第一次步进前加载。
      spindle_set_speed(st.exec_segment->spindle_pwm);

    } else {
      // 段缓冲区为空。关闭。
      st_go_idle();
      // 确保在完成速率控制运动时，PWM设置正确。
      if (st.exec_block->is_pwm_rate_adjusted) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
      system_set_exec_state_flag(EXEC_CYCLE_STOP); // 标记主程序以结束循环
      return; // 没有其他事情可以做，退出。
    }
  }

  // 检查探测状态。
  if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }

  // 重置步出位。
  st.step_outbits = 0;

  // 执行通过Bresenham线算法的步进位移配置文件
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1 << X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << X_DIRECTION_BIT)) { sys_position[X_AXIS]--; }
    else { sys_position[X_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1 << Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << Y_DIRECTION_BIT)) { sys_position[Y_AXIS]--; }
    else { sys_position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1 << Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << Z_DIRECTION_BIT)) { sys_position[Z_AXIS]--; }
    else { sys_position[Z_AXIS]++; }
  }

#ifdef A_AXIS
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_a += st.steps[A_AXIS];
  #else
    st.counter_a += st.exec_block->steps[A_AXIS];
  #endif
  if (st.counter_a > st.exec_block->step_event_count) {
    st.step_outbits |= (1 << A_STEP_BIT);
    st.counter_a -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << A_DIRECTION_BIT)) { sys_position[A_AXIS]--; }
    else { sys_position[A_AXIS]++; }
  }
#endif
#ifdef B_AXIS
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_b += st.steps[B_AXIS];
  #else
    st.counter_b += st.exec_block->steps[B_AXIS];
  #endif
  if (st.counter_b > st.exec_block->step_event_count) {
    st.step_outbits |= (1 << B_STEP_BIT);
    st.counter_b -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << B_DIRECTION_BIT)) { sys_position[B_AXIS]--; }
    else { sys_position[B_AXIS]++; }
  }
#endif
#ifdef C_AXIS
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_c += st.steps[C_AXIS];
  #else
    st.counter_c += st.exec_block->steps[C_AXIS];
  #endif
  if (st.counter_c > st.exec_block->step_event_count) {
    st.step_outbits |= (1 << C_STEP_BIT);
    st.counter_c -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1 << C_DIRECTION_BIT)) { sys_position[C_AXIS]--; }
    else { sys_position[C_AXIS]++; }
  }
#endif

  // 在归位周期中，锁定并防止所需轴移动。
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }

  st.step_count--; // 减少步事件计数
  if (st.step_count == 0) {
    // 段完成。丢弃当前段并推进段索引。
    st.exec_segment = NULL;
    if (++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.step_outbits ^= step_port_invert_mask;  // 应用步进端口反转掩码
  busy = false;
}


/* 步进端口重置中断：Timer0 OVF中断处理步进脉冲的下降沿。
   这应该总是在下一个Timer1 COMPA中断之前触发，并独立完成，
   如果在完成移动后禁用Timer1。
   注意：串行和步进中断之间的中断冲突可能会导致延迟
   几微秒，如果它们在彼此之前执行。这不是大问题，但如果添加另一个高频
   异步中断到Grbl，可能会导致高步频下的问题。
*/
// 此中断由ISR_TIMER1_COMPAREA启用，当它设置电机端口位以执行
// 步进时。此ISR在短时间（settings.pulse_microseconds）后重置电机端口
// 完成一个步进周期。
ISR(TIMER0_OVF_vect)
{
  // 重置步进引脚（保持方向引脚不变）
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK);
  TCCR0B = 0; // 禁用Timer0，以防在不需要时重新进入此中断。
}
#ifdef STEP_PULSE_DELAY
  // 此中断仅在启用STEP_PULSE_DELAY时使用。在此，步进脉冲
  // 在STEP_PULSE_DELAY时间段结束后启动。ISR TIMER2_OVF中断
  // 然后将在适当的settings.pulse_microseconds后触发，像正常操作一样。
  // 方向、步进脉冲和步进完成事件之间的新定时在
  // st_wake_up()例程中设置。
  ISR(TIMER0_COMPA_vect)
  {
    STEP_PORT = st.step_bits; // 开始步进脉冲。
  }
#endif


// 生成用于步进器中断驱动程序的步进和方向端口反转掩码。
void st_generate_step_dir_invert_masks()
{
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx = 0; idx < N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask, bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    if (bit_istrue(settings.dir_invert_mask, bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); }
  }
}


// 重置并清除步进器子系统变量
void st_reset()
{
  // 初始化步进驱动程序空闲状态。
  st_go_idle();

  // 初始化步进算法变量。
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // 由段缓冲区使用的规划块指针
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // 空 = 尾部
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; // 初始化方向位为默认值。

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

  // 配置计时器1：步进驱动程序中断
  TCCR1B &= ~(1 << WGM13); // 波形生成 = 0100 = CTC
  TCCR1B |= (1 << WGM12);
  TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0)); // 断开OC1输出
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // 在st_go_idle()中设置。
  // TIMSK1 &= ~(1<<OCIE1A);  // 在st_go_idle()中设置。

  // 配置计时器0：步进端口重置中断
  TIMSK0 &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0)); // 断开OC0输出和OVF中断。
  TCCR0A = 0; // 正常操作
  TCCR0B = 0; // 禁用Timer0，直到需要
  TIMSK0 |= (1 << TOIE0); // 启用Timer0溢出中断
  #ifdef STEP_PULSE_DELAY
    TIMSK0 |= (1 << OCIE0A); // 启用Timer0比较匹配A中断
  #endif
}


// 当执行块由新计划更新时由planner_recalculate()调用。
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { // 如果在新块开始时则忽略。
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed * prep.current_speed; // 更新进入速度。
    pl_block = NULL; // 标记st_prep_segment()以加载和检查活动速度配置文件。
  }
}


// 增加步进段缓冲区块数据环形缓冲区的索引。
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}

#ifdef PARKING_ENABLE
  // 更改步进段缓冲区的运行状态，以执行特殊的停车动作。
  void st_parking_setup_buffer()
  {
    // 如有必要，存储部分完成块的步进执行数据。
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
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

  // 在停车动作后恢复步进段缓冲区到正常运行状态。
  void st_parking_restore_buffer()
  {
    // 如有必要，恢复部分完成块的步进执行数据和标志。
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      st_prep_block = &st_block_buffer[prep.last_st_block_index];
      prep.st_block_index = prep.last_st_block_index;
      prep.steps_remaining = prep.last_steps_remaining;
      prep.dt_remainder = prep.last_dt_remainder;
      prep.step_per_mm = prep.last_step_per_mm;
      prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
      prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm; // 重新计算此值。
    } else {
      prep.recalculate_flag = false;
    }
    pl_block = NULL; // 设置为重新加载下一个块。
  }
#endif

/* 准备步进段缓冲区。不断从主程序调用。

   段缓冲区是步进算法执行步骤和规划器生成的速度轮廓之间的中介缓冲区接口。
   步进算法仅在段缓冲区内执行步骤，并在主程序中从规划器缓冲区的第一个块中“检查出”步骤时填充该缓冲区。
   这使得步进执行和规划优化过程原子化，并彼此保护。
   从规划器缓冲区“检查出”的步骤数量和段缓冲区中的段数量大小和计算，使得主程序中的任何操作所需的时间
   不会超过步进算法在重新填充缓冲区之前清空它所需的时间。
   目前，段缓冲区保守地保存大约 40-50 毫秒的步骤。
   注意：计算单位为步骤、毫米和分钟。
*/
void st_prep_buffer()
{
  // 在暂停状态且没有暂停运动要执行时，阻塞步进准备缓冲区。
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }

  while (segment_buffer_tail != segment_next_head) { // 检查是否需要填充缓冲区。

    // 确定是否需要加载新规划块或是否需要重新计算块。
    if (pl_block == NULL) {

      // 查询规划器以获取排队的块
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { pl_block = plan_get_system_motion_block(); }
      else { pl_block = plan_get_current_block(); }
      if (pl_block == NULL) { return; } // 没有规划器块。退出。

      // 检查是否仅需要重新计算速度轮廓或加载新块。
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
          else { prep.recalculate_flag = false; }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {

        // 加载块的 Bresenham 步进数据。
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        // 准备并复制来自新规划块的 Bresenham 算法段数据，以便在段缓冲区完成规划块时
        // 可以丢弃它，而步进 ISR 仍在执行它。
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx]; }
          st_prep_block->step_event_count = pl_block->step_event_count;
        #else
          // 启用 AMASS 时，只需将所有 Bresenham 数据按最大 AMASS
          // 水平左移，以便我们不会在算法中的任何地方除以原始数据。
          // 如果原始数据被除以，我们可能会因整数舍入而丢失一步。
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        // 初始化段缓冲区数据以生成段。
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0; // 为新段块重置

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          // 在保持状态中加载新块。覆盖规划块条目速度以强制减速。
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrt(pl_block->entry_speed_sqr);
        }
        
        // 设置激光模式变量。调整 PWM 速率的运动将始终在电机关闭的情况下完成运动。
        st_prep_block->is_pwm_rate_adjusted = false;
        if (settings.flags & BITFLAG_LASER_MODE) {
          if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) { 
            // 预计算逆编程速率，以加快每个步进段的 PWM 更新。
            prep.inv_rate = 1.0/pl_block->programmed_rate;
            st_prep_block->is_pwm_rate_adjusted = true; 
          }
        }
      }

      /* ---------------------------------------------------------------------------------
         根据新规划块的进入和退出速度计算速度轮廓，或重新计算部分完成规划块的轮廓
         如果规划器已更新它。对于强制减速命令，例如来自进给保持的命令，
         将覆盖规划器速度并减速到目标退出速度。
      */
      prep.mm_complete = 0.0; // 默认速度轮廓在离块末端 0.0mm 处完成。
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { // [强制减速到零速度]
        // 为进行中的进给保持计算速度轮廓参数。此轮廓覆盖
        // 规划块轮廓，强制减速到零速度。
        prep.ramp_type = RAMP_DECEL;
        // 相对于块末端计算减速距离。
        float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < 0.0) {
          // 在整个规划块中减速。进给保持的结束不在此块中。
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        } else {
          prep.mm_complete = decel_dist; // 进给保持结束。
          prep.exit_speed = 0.0;
        }
      } else { // [正常操作]
        // 计算或重新计算准备的规划块的速度轮廓参数。
        prep.ramp_type = RAMP_ACCEL; // 初始化为加速坡道。
        prep.accelerate_until = pl_block->millimeters;

        float exit_speed_sqr;
        float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0; // 在系统运动结束时强制停止。
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrt(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
        float nominal_speed_sqr = nominal_speed*nominal_speed;
        float intersect_distance =
            0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // 仅在覆盖减少期间发生。
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0) { // 仅减速。
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // 计算覆盖块退出速度，因为它与规划器退出速度不匹配。
            prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // 标记为将下一个块加载为减速覆盖。

            // TODO: 确定减速仅的参数处理。
            // 这可能很棘手，因为进入速度将是当前速度，就像在进给保持中一样。
            // 还要关注处理接近零速度时的问题。

          } else {
            // 减速到巡航或巡航-减速类型。保证与更新计划相交。
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
            prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
        } else if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->millimeters) { // 可能是梯形或三角形类型
            // 注意：对于加速-巡航和仅巡航类型，后续计算将为 0.0。
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // 梯形类型
              prep.maximum_speed = nominal_speed;
              if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                // 巡航-减速或仅巡航类型。
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // 完整的梯形或加速-巡航类型
                prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
              }
            } else { // 三角形类型
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }
          } else { // 仅减速类型
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
          }
        } else { // 仅加速类型
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }
      
      bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // 每次更新块时强制更新。
    }
    
    // 初始化新段
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // 将新段指向当前段数据块。
    prep_segment->st_block_index = prep.st_block_index;


   /*------------------------------------------------------------------------------------
        通过确定在段时间 DT_SEGMENT 内行驶的总距离，计算该新段的平均速度。
      以下代码首先尝试根据当前的斜坡条件创建一个完整的段。如果在斜坡状态变化时段时间不完整，代码将继续循环遍历正在进行的斜坡状态，以填补剩余的段执行时间。然而，如果一个不完整的段在速度轮廓的末尾终止，则该段被视为完成，尽管其执行时间少于 DT_SEGMENT。
        速度轮廓始终假定经过斜坡序列：加速斜坡、巡航状态和减速斜坡。每个斜坡的行驶距离可以从零到块的长度。速度轮廓可以在计划块的末尾（典型）或在强制减速结束时在块中间结束，例如在进给保持时。
------------------------------------------------------------------------------------*/
float dt_max = DT_SEGMENT; // 最大段时间
float dt = 0.0; // 初始化段时间
float time_var = dt_max; // 时间工作变量
float mm_var; // mm-距离工作变量
float speed_var; // 速度工作变量
float mm_remaining = pl_block->millimeters; // 从块末尾开始的新段距离
float minimum_mm = mm_remaining - prep.req_mm_increment; // 保证至少有一步
if (minimum_mm < 0.0) { minimum_mm = 0.0; }

do {
    switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
            speed_var = pl_block->acceleration * time_var;
            mm_var = time_var * (prep.current_speed - 0.5 * speed_var);
            mm_remaining -= mm_var;
            if ((mm_remaining < prep.accelerate_until) || (mm_var <= 0)) {
                // 仅对于减速覆盖类型的巡航或巡航减速。
                mm_remaining = prep.accelerate_until; // 注意：在 EOB 时为 0.0
                time_var = 2.0 * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                prep.ramp_type = RAMP_CRUISE;
                prep.current_speed = prep.maximum_speed;
            } else { // 中途减速覆盖斜坡
                prep.current_speed -= speed_var;
            }
            break;
        case RAMP_ACCEL:
            // 注意：加速斜坡仅在第一次 do-while 循环中计算。
            speed_var = pl_block->acceleration * time_var;
            mm_remaining -= time_var * (prep.current_speed + 0.5 * speed_var);
            if (mm_remaining < prep.accelerate_until) { // 加速斜坡结束
                // 加速巡航、加速减速斜坡交界，或块末尾
                mm_remaining = prep.accelerate_until; // 注意：在 EOB 时为 0.0
                time_var = 2.0 * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
                else { prep.ramp_type = RAMP_CRUISE; }
                prep.current_speed = prep.maximum_speed;
            } else { // 仅加速
                prep.current_speed += speed_var;
            }
            break;
        case RAMP_CRUISE:
            // 注意：mm_var 用于保留不完整段 time_var 计算的最后 mm_remaining。
            // 注意：如果 maximum_speed * time_var 值过低，舍入可能导致 mm_var 不变化。为
            //   防止这种情况，只需在规划中强制执行最小速度阈值。
            mm_var = mm_remaining - prep.maximum_speed * time_var;
            if (mm_var < prep.decelerate_after) { // 巡航结束
                // 巡航减速交界或块末尾。
                time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                mm_remaining = prep.decelerate_after; // 注意：在 EOB 时为 0.0
                prep.ramp_type = RAMP_DECEL;
            } else { // 仅巡航
                mm_remaining = mm_var;
            }
            break;
        default: // case RAMP_DECEL:
            // 注意：mm_var 用作杂项工作变量，以防在接近零速度时出现错误。
            speed_var = pl_block->acceleration * time_var; // 用作增量速度 (mm/min)
            if (prep.current_speed > speed_var) { // 检查是否在零速度或以下
                // 计算从段末到块末的距离
                mm_var = mm_remaining - time_var * (prep.current_speed - 0.5 * speed_var); // (mm)
                if (mm_var > prep.mm_complete) { // 典型情况。在减速斜坡中
                    mm_remaining = mm_var;
                    prep.current_speed -= speed_var;
                    break; // 段完成。退出 switch-case 语句。继续 do-while 循环。
                }
            }
            // 否则，在块末尾或强制减速结束时
            time_var = 2.0 * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
            mm_remaining = prep.mm_complete;
            prep.current_speed = prep.exit_speed;
    }
    dt += time_var; // 将计算的斜坡时间加到总段时间
    if (dt < dt_max) { time_var = dt_max - dt; } // **不完整** 在斜坡交界处
    else {
        if (mm_remaining > minimum_mm) { // 检查非常缓慢的段是否没有零步骤
            // 增加段时间以确保段中至少有一步。覆盖并循环
            // 通过距离计算直到 minimum_mm 或 mm_complete。
            dt_max += DT_SEGMENT;
            time_var = dt_max - dt;
        } else {
            break; // **完成** 退出循环。段执行时间达到最大。
        }
    }
} while (mm_remaining > prep.mm_complete); // **完成** 退出循环。轮廓完成。

/* -----------------------------------------------------------------------------------
      计算步骤段的主轴速度 PWM 输出
*/

if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
    if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
        float rpm = pl_block->spindle_speed;
        // 注意：进给和快速覆盖独立于 PWM 值，不改变激光功率/速率。
        if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
        // 如果 current_speed 为零，则可能需要 rpm_min * (100/MAX_SPINDLE_SPEED_OVERRIDE)
        // 但这只是在运动中的瞬时值。可能根本不重要。
        prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
    } else { 
        sys.spindle_speed = 0.0;
        prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
    }
    bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
}
prep_segment->spindle_pwm = prep.current_spindle_pwm; // 重新加载段 PWM 值

/* -----------------------------------------------------------------------------------
      计算段步速、要执行的步数，并应用必要的速率修正。
      注意：步数是通过将剩余的毫米距离直接转换为标量来计算的，而不是逐步记录每个段执行的步数。
      这有助于消除多个加法的浮点舍入问题。
      然而，由于浮点数仅有 7.2 位有效数字，因此长移动具有极高步数可能会超出浮点的精度，从而导致丢失步骤。
      幸运的是，这种情况在 Grbl 支持的 CNC 机器中极不可能且不现实（即，轴行程超过 10 米，200 步/mm）。
*/
float step_dist_remaining = prep.step_per_mm * mm_remaining; // 将 mm_remaining 转换为步骤
float n_steps_remaining = ceil(step_dist_remaining); // 向上取整当前剩余步骤
float last_n_steps_remaining = ceil(prep.steps_remaining); // 向上取整最后剩余步骤
prep_segment->n_step = last_n_steps_remaining - n_steps_remaining; // 计算要执行的步骤数。

// 如果我们在进给保持的末尾并且没有要执行的步骤，则退出。
if (prep_segment->n_step == 0) {
    if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // 少于一步的减速至零速度，但已经非常接近。AMASS
        // 需要执行完整的步骤。所以，直接退出。
        bit_true(sys.step_control, STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
            if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // 段未生成，但当前步骤数据仍然保留。
    }
}

// 计算段步速。由于步骤是整数，而毫米行驶距离不是，因此
// 每个段的结束可能会有不同幅度的部分步骤未执行，因为步进 ISR 需要整个步骤。
为了补偿，我们跟踪上一个段部分步骤的执行时间，并简单地
应用它与当前段的部分步骤距离，以便细微地
调整整个段速率以保持步骤输出精确。这些速率调整
通常非常小，不会对性能产生不利影响，但确保 Grbl
输出由规划器计算的确切加速度和速度轮廓。
dt += prep.dt_remainder; // 应用上一个段部分步骤执行时间
float inv_rate = dt / (last_n_steps_remaining - step_dist_remaining); // 计算调整后的步速逆

// 计算准备段的每步 CPU 周期数。
uint32_t cycles = ceil((TICKS_PER_MICROSECOND * 1000000 * 60) * inv_rate); // (cycles/step)

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    // 计算步骤定时和多轴平滑级别。
    // 注意：AMASS 在每个级别过驱动定时器，因此只需要一个预分频器。
    if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
    else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
    }
    if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
    else { prep_segment->cycles_per_tick = 0xffff; } // 仅设置最慢速度。
#else
    // 计算步骤定时和普通步生成的定时器预分频器。
    if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // 预分频器：0
        prep_segment->cycles_per_tick = cycles;
    } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // 预分频器：8
        prep_segment->cycles_per_tick = cycles >> 3;
    } else {
        prep_segment->prescaler = 3; // 预分频器：64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
            prep_segment->cycles_per_tick = cycles >> 6;
        } else { // 仅设置最慢速度。 (约 4 步/秒)
            prep_segment->cycles_per_tick = 0xffff;
        }
    }
#endif

// 段完成！递增段缓冲区索引，以便步进 ISR 可以立即执行它。
segment_buffer_head = segment_next_head;
if (++segment_next_head == SEGMENT_BUFFER_SIZE) { segment_next_head = 0; }

// 更新适当的规划和段数据。
pl_block->millimeters = mm_remaining;
prep.steps_remaining = n_steps_remaining;
prep.dt_remainder = (n_steps_remaining - step_dist_remaining) * inv_rate;

// 检查退出条件并标记以加载下一个规划块。
if (mm_remaining == prep.mm_complete) {
    // 规划块结束或强制终止。没有更多距离可以执行。
    if (mm_remaining > 0.0) { // 在强制终止时
        // 重置准备参数以恢复，然后退出。允许步进 ISR 完成
        // 段队列，实时协议将在接收到 ISR 的
        // 周期停止标志后设置新状态。准备段在此之前被阻塞。
        bit_true(sys.step_control, STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
            if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // 退出！
    } else { // 规划块结束
        // 规划块完成。所有步骤将在段缓冲区中执行。
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
            bit_true(sys.step_control, STEP_CONTROL_END_MOTION);
            return;
        }
        pl_block = NULL; // 设置指针以指示检查和加载下一个规划块。
        plan_discard_current_block();
    }
}

}

// 由实时状态报告调用以获取当前正在执行的速度。此值
// 然而，并不完全是当前速度，而是段缓冲区中
// 上一个步骤段中计算的速度。它将始终滞后于段块数 (-1)
// 除以每秒加速度滴答数（以秒为单位）。
float st_get_realtime_rate()
{
    if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)){
        return prep.current_speed;
    }
    return 0.0f;
}
