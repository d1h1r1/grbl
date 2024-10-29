/*
  planner.h - 缓存运动命令并管理加速曲线计划
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#ifndef planner_h
#define planner_h


// 在任何给定时间内，计划中可以包含的线性运动数量
#ifndef BLOCK_BUFFER_SIZE
  #define BLOCK_BUFFER_SIZE 36
#endif

// 从规划器返回的状态消息。
#define PLAN_OK true
#define PLAN_EMPTY_BLOCK false

// 定义规划器数据条件标志。用于表示块的运行条件。
#define PL_COND_FLAG_RAPID_MOTION      bit(0)
#define PL_COND_FLAG_SYSTEM_MOTION     bit(1) // 单一运动。绕过规划器状态。由归位/停车使用。
#define PL_COND_FLAG_NO_FEED_OVERRIDE  bit(2) // 运动不考虑进给覆盖。
#define PL_COND_FLAG_INVERSE_TIME      bit(3) // 当设置时，将进给速率值解释为逆时间。
#define PL_COND_FLAG_SPINDLE_CW        bit(4)
#define PL_COND_FLAG_SPINDLE_CCW       bit(5)
#define PL_COND_FLAG_COOLANT_FLOOD     bit(6)
#define PL_COND_FLAG_COOLANT_MIST      bit(7)
#define PL_COND_MOTION_MASK    (PL_COND_FLAG_RAPID_MOTION|PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE)
#define PL_COND_ACCESSORY_MASK (PL_COND_FLAG_SPINDLE_CW|PL_COND_FLAG_SPINDLE_CCW|PL_COND_FLAG_COOLANT_FLOOD|PL_COND_FLAG_COOLANT_MIST)

// 此结构存储 g-code 块运动的线性运动及其关键的“标称”值
// 按照源 g-code 中的规定。
typedef struct {
  // Bresenham 算法用于跟踪线的字段
  // 注意：由步进器算法正确执行块。请勿更改这些值。
  uint32_t steps[N_AXIS];    // 每个轴的步数
  uint32_t step_event_count; // 完成此块所需的最大步轴计数和步数。
  uint8_t direction_bits;    // 此块的方向位设置（引用 config.h 中的 *_DIRECTION_BIT）

  // 块条件数据以确保根据状态和覆盖进行正确执行。
  uint8_t condition;      // 定义块运行条件的块位标志变量。复制自 pl_line_data。
  int32_t line_number;  // 实时报告的块行号。复制自 pl_line_data。

  // 运动规划器用于管理加速的字段。某些值可能在执行特殊运动案例时由步进模块更新
  // 以进行重新规划。
  float entry_speed_sqr;     // 块连接处当前计划的入速（mm/min）^2
  float max_entry_speed_sqr; // 基于连接限制和相邻标称速度的最小值的最大允许入速
                             // （mm/min）^2
  float acceleration;        // 轴限制调整后的线加速度（mm/min^2）。不改变。
  float millimeters;         // 此块在执行中的剩余距离（mm）。
                             // 注意：此值可能在执行过程中由步进算法更改。

  // 规划器在发生变化时使用的速率限制数据。
  float max_junction_speed_sqr; // 基于方向向量的连接入速限制（mm/min）^2
  float rapid_rate;             // 此块方向的轴限制调整后的最大速率（mm/min）
  float programmed_rate;        // 此块的编程速率（mm/min）。

  // 用于主轴覆盖和恢复方法的存储主轴速度数据。
  float spindle_speed;    // 块主轴速度。复制自 pl_line_data。
} plan_block_t;

// 规划器数据原型。传递新运动给规划器时必须使用。
typedef struct {
  float feed_rate;          // 线性运动所需的进给速率。如果是快速运动，则忽略该值。
  float spindle_speed;      // 线性运动的所需主轴速度。
  int32_t line_number;    // 执行时要报告的所需行号。
  uint8_t condition;        // 指示规划器条件的位标志变量。请参阅上面的定义。
} plan_line_data_t;

// 初始化并重置运动计划子系统
void plan_reset(); // 重置所有
void plan_reset_buffer(); // 仅重置缓冲区。

// 将新的线性运动添加到缓冲区。target[N_AXIS] 是以毫米为单位的带符号绝对目标位置。
// 进给速率指定运动的速度。如果进给速率被反转，则进给速率被视为“频率”，
// 将在 1/feed_rate 分钟内完成操作。
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data);

// 当当前块不再需要时调用。丢弃该块并释放内存
// 以便用于新块。
void plan_discard_current_block();

// 获取特殊系统运动案例的规划器块。（停车/归位）
plan_block_t *plan_get_system_motion_block();

// 获取当前块。如果缓冲区为空则返回 NULL
plan_block_t *plan_get_current_block();

// 由步进段缓冲区定期调用。主要由规划器内部使用。
uint8_t plan_next_block_index(uint8_t block_index);

// 在计算执行块速度曲线时由步进段缓冲区调用。
float plan_get_exec_block_exit_speed_sqr();

// 在主程序进行规划计算和步进段缓冲区初始化期间调用。
float plan_compute_profile_nominal_speed(plan_block_t *block);

// 在基于运动的覆盖变化后重新计算缓冲运动的配置参数。
void plan_update_velocity_profile_parameters();

// 重置规划器位置向量（以步数计）
void plan_sync_position();

// 用部分完成的块重新初始化计划
void plan_cycle_reinitialize();

// 返回规划器缓冲区中可用块的数量。
uint8_t plan_get_block_buffer_available();

// 返回规划器缓冲区中活动块的数量。
// 注意：已弃用。除非在 config.h 中启用经典状态报告，否则不使用。
uint8_t plan_get_block_buffer_count();

// 返回块环缓冲区的状态。如果缓冲区已满，则返回 true。
uint8_t plan_check_full_buffer();

void plan_get_planner_mpos(float *target);

#endif
