/*
  planner.c - 缓冲运动命令并管理加速曲线规划
  Grbl的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud
  版权所有 (c) 2011 Jens Geisler

  Grbl 是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改
  它，许可证的版本为 3，或者（根据您的选择）任何后续版本。

  Grbl 以希望它有用的方式分发，
  但不提供任何保证；甚至没有对
  适销性或适合特定目的的隐含保证。请参见
  GNU 通用公共许可证的更多详细信息。

  您应该已收到 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // 运动指令的环形缓冲区
static uint8_t block_buffer_tail;     // 当前处理的块索引
static uint8_t block_buffer_head;     // 下一个要推入的块索引
static uint8_t next_buffer_head;      // 下一个缓冲区头索引
static uint8_t block_buffer_planned;  // 最优规划块的索引

// 定义规划变量
typedef struct {
  int32_t position[N_AXIS];          // 工具在绝对步长下的规划位置。与 G-code 位置分开保存
                                     // 以处理需要多个线路运动的移动，
                                     // 即弧线、固定循环和反向间隙补偿。
  float previous_unit_vec[N_AXIS];   // 前一个路径线段的单位向量
  float previous_nominal_speed;  // 前一个路径线段的名义速度
} planner_t;
static planner_t pl;


// 返回环形缓冲区中下一个块的索引。也由步进段缓冲区调用。
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// 返回环形缓冲区中前一个块的索引
static uint8_t plan_prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*                            规划速度定义
                                     +--------+   <- current->nominal_speed
                                    /          \
         current->entry_speed ->   +            \
                                   |             + <- next->entry_speed（即退出速度）
                                   +-------------+
                                       时间 -->

  根据以下基本准则重新计算运动规划：

    1. 按逆序逐块检查每个可行块并计算连接速度
        （即 current->entry_speed），以确保：
      a. 任何连接速度不超过预先计算的最大连接速度限制或相邻块的名义速度。
      b. 块的入口速度不能超过根据其退出速度（next->entry_speed）
         反向计算出的速度，并且在块运动距离上具有最大允许减速。
      c. 最后（或最新添加的）块从完全停止（退出速度为零）开始规划。
    2. 按时间顺序（正向）逐块检查每个块，并在以下情况下降低连接速度值：
      a. 退出速度超过根据其入口速度和最大允许加速度计算得出的速度。

  当这些阶段完成后，规划器将最大化所有规划块的速度曲线，其中每个块都在其最大允许加速度限制下运行。
  换句话说，对于规划器中的所有块，规划是最优的，无法进一步提高速度。如果向缓冲区添加新块，将根据上述准则重新计算新规划。

  为了提高这些准则的计算效率，创建了一组规划块指针，以指示在正常操作中，当新块流入并添加到规划器缓冲区时，规划器准则无法逻辑上进行进一步更改或改进的停止计算点。
  例如，如果规划器中的一组连续块已被规划并且其连接速度达到最大值（或第一个规划块），则添加到规划器缓冲区的任何新块将不会改变其中的速度曲线。因此，我们不再需要计算它们。或者，如果来自规划器中第一个块的连续块（或最佳停止计算点）都在加速，它们都是最优的，不能被添加到规划器缓冲区的新块改变，因为这只会在达到最大连接速度之前进一步增加到时间顺序块的计划速度。然而，如果规划的操作条件因不常用的进给保持或进给速率覆盖而改变，停止计算指针将被重置，并且整个规划将根据一般准则重新计算。

  规划缓冲区索引映射：
  - block_buffer_tail：指向规划缓冲区的开始。第一个被执行或正在执行的块。
  - block_buffer_head：指向缓冲区中最后一个块之后的缓冲块。用于指示缓冲区是满还是空。与标准环形缓冲区描述相同，该块始终为空。
  - next_buffer_head：指向缓冲区头块之后的下一个规划缓冲区块。当等于缓冲区尾时，指示缓冲区已满。
  - block_buffer_planned：指向正常流操作条件下最后一个最佳规划块之后的第一个缓冲块。用于规划优化，通过避免在添加新块时重新计算不变的规划缓冲区部分，如上所述。此外，该块永远不会小于 block_buffer_tail，并且在遇到 plan_discard_current_block() 例程时将始终向前推进并保持此要求。

  注意：由于规划器仅在规划缓冲区内计算，因此某些具有许多短线段的运动，如 G2/G3 弧线或复杂曲线，可能看起来移动缓慢。这是因为整个缓冲区内的总行驶距离不足以加速到名义速度，然后在缓冲区结束时减速至完全停止，正如准则所述。如果发生这种情况并且变得令人烦恼，有一些简单的解决方案：(1) 最大化机器加速度。规划器将能够在相同的总行驶距离内计算更高的速度曲线。(2) 最大化每个块的线路运动距离到所需的公差。规划器可用的总行驶距离越多，它的速度就越快。(3) 最大化规划器缓冲区大小。这也将增加规划器计算的总行驶距离。这也增加了规划器为计算最佳规划而必须执行的计算次数，因此请谨慎选择。Arduino 328p 的内存已经达到了最大值，但未来的 ARM 版本应该有足够的内存和速度，能够处理多达一百个或更多的前瞻性块。

*/

static void planner_recalculate()
{
  // 初始化块索引为规划缓冲区中的最后一个块。
  uint8_t block_index = plan_prev_block_index(block_buffer_head);

  // 退出。如果只有一个可规划的块，则无法执行任何操作。
  if (block_index == block_buffer_planned) { return; }

  // 反向计算：从缓冲区中的最后一个块开始粗略最大化所有可能的减速曲线。 
  // 当达到最后一个最优规划块或尾指针时停止规划。
  // 注意：正向计算将稍后细化和纠正反向计算，以创建最优规划。
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  // 计算缓冲区中最后一个块的最大入口速度，退出速度始终为零。
  current->entry_speed_sqr = min(current->max_entry_speed_sqr, 2 * current->acceleration * current->millimeters);

  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) { // 缓冲区中只有两个可规划块。反向计算完成。
    // 检查第一个块是否是尾块。如果是，通知步进器更新其当前参数。
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else { // 三个或更多可规划块
    while (block_index != block_buffer_planned) {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // 检查下一个块是否是尾块（=规划块）。如果是，更新当前步进器参数。
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }

      // 根据当前块的退出速度计算最大入口速度。
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }

  // 正向计算：从规划指针开始正向规划加速曲线。
  // 还扫描最优规划的断点并适当地更新规划指针。
  next = &block_buffer[block_buffer_planned]; // 从缓冲区规划指针开始
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

    // 在正向计算中检测到的任何加速自动将最优规划指针向前移动，因为之前的所有都是最优的。
    // 换句话说，逻辑上从缓冲区尾到规划指针之间的任何内容都无法改善规划。
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
      // 如果为真，当前块为全加速，且我们可以将规划指针向前移动。
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // 始终 <= max_entry_speed_sqr。反向计算设置此值。
        block_buffer_planned = block_index; // 设置最优规划指针。
      }
    }

    // 任何设置为其最大入口速度的块也会在缓冲区的这一点上创建最优规划。
    // 当规划被缓冲区的开头和最大入口速度或两个最大入口速度括起来时，之间的每个块都逻辑上无法进一步改善。
    // 因此，我们不再需要重新计算它们。
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index(block_index);
  }
}


void plan_reset()
{
  memset(&pl, 0, sizeof(planner_t)); // 清除规划器结构体
  plan_reset_buffer();
}


void plan_reset_buffer()
{
  block_buffer_tail = 0;
  block_buffer_head = 0; // 空 = 尾
  next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}


void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail) { // 丢弃非空缓冲区。
    uint8_t block_index = plan_next_block_index(block_buffer_tail);
    // 如果遇到则推送 block_buffer_planned 指针。
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


// 返回系统运动使用的规划缓冲区块的地址。由段生成器调用。
plan_block_t *plan_get_system_motion_block()
{
  return(&block_buffer[block_buffer_head]);
}


// 返回第一个规划块的地址（如果可用）。由各种主程序功能调用。
plan_block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } // 缓冲区为空
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed_sqr()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( block_buffer[block_index].entry_speed_sqr );
}


// 返回块环形缓冲区的可用状态。如果满，返回真。
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


// 根据运行条件和覆盖值计算并返回块的名义速度。
// 注意：所有系统运动命令，如归位/停车，不受覆盖的影响。
float plan_compute_profile_nominal_speed(plan_block_t *block)
{
  float nominal_speed = block->programmed_rate;
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { nominal_speed *= (0.01 * sys.r_override); }
  else {
    if (!(block->condition & PL_COND_FLAG_NO_FEED_OVERRIDE)) { nominal_speed *= (0.01 * sys.f_override); }
    if (nominal_speed > block->rapid_rate) { nominal_speed = block->rapid_rate; }
  }
  if (nominal_speed > MINIMUM_FEED_RATE) { return(nominal_speed); }
  return(MINIMUM_FEED_RATE);
}


// 计算并更新块的最大入口速度（平方），基于连接的前后名义速度和最大连接速度的最小值。
static void plan_compute_profile_parameters(plan_block_t *block, float nominal_speed, float prev_nominal_speed)
{
  // 根据连接速度和相邻名义速度的最小值计算连接的最大入口速度。
  if (nominal_speed > prev_nominal_speed) { block->max_entry_speed_sqr = prev_nominal_speed * prev_nominal_speed; }
  else { block->max_entry_speed_sqr = nominal_speed * nominal_speed; }
  if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) { block->max_entry_speed_sqr = block->max_junction_speed_sqr; }
}


// 在基于运动的覆盖变化后重新计算缓冲运动的参数。
void plan_update_velocity_profile_parameters()
{
  uint8_t block_index = block_buffer_tail;
  plan_block_t *block;
  float nominal_speed;
  float prev_nominal_speed = SOME_LARGE_VALUE; // 设置为高值以便计算第一个块的名义速度。
  while (block_index != block_buffer_head) {
    block = &block_buffer[block_index];
    nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
    prev_nominal_speed = nominal_speed;
    block_index = plan_next_block_index(block_index);
  }
  pl.previous_nominal_speed = prev_nominal_speed; // 更新上一个名义速度，以便于下一个传入块。
}


/* 将新的线性运动添加到缓冲区。 target[N_AXIS] 是以毫米为单位的带符号绝对目标位置。
   进给速率指定运动的速度。如果进给速率被反转，则进给速率表示“频率”，将在 1/进给速率分钟内完成操作。
   所有传递给规划器的位置数据必须基于机器位置，以使规划器独立于任何坐标系变化和偏移，这些由 G-code 解析器处理。
   注意：假设缓冲区可用。缓冲区检查在更高层次的 motion_control 中处理。
   换句话说，缓冲区头从不等于缓冲区尾。此外，进给速率输入值以三种方式使用：如果 invert_feed_rate 为 false，则作为正常进给速率；如果 invert_feed_rate 为 true，则作为反向时间；如果 feed_rate 值为负（并且 invert_feed_rate 始终为 false），则作为寻址/快速速率。
   系统运动条件告诉规划器在始终未使用的块缓冲区头中规划运动。它避免更改规划器状态并保留缓冲区，以确保后续的 G-code 运动仍能正确规划，同时步进模块仅指向块缓冲区头以执行特殊的系统运动。 */

uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data)
{
  // 准备并初始化新块。复制相关的 pl_data 以供块执行。
  plan_block_t *block = &block_buffer[block_buffer_head];
  memset(block,0,sizeof(plan_block_t)); // 将所有块值置零。
  block->condition = pl_data->condition;
  block->spindle_speed = pl_data->spindle_speed;
  block->line_number = pl_data->line_number;

  // 计算并存储初始移动距离数据。
  int32_t target_steps[N_AXIS], position_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;

  // 根据计划的运动类型复制位置数据。
  if (block->condition & PL_COND_FLAG_SYSTEM_MOTION) { 
    #ifdef COREXY
      position_steps[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      position_steps[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      position_steps[Z_AXIS] = sys_position[Z_AXIS];
    #else
      memcpy(position_steps, sys_position, sizeof(sys_position)); 
    #endif
  } else { memcpy(position_steps, pl.position, sizeof(pl.position)); }

  #ifdef COREXY
    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
    block->steps[A_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) + (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
    block->steps[B_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) - (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
  #endif

  for (idx=0; idx<N_AXIS; idx++) {
    // 计算目标位置的绝对步数、每个轴的步数，并确定最大步数事件。
    // 还计算每个轴的移动距离并准备单位向量计算。
    // 注意：计算的是真正的距离，基于转换后的步数值。
    #ifdef COREXY
      if ( !(idx == A_MOTOR) && !(idx == B_MOTOR) ) {
        target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
        block->steps[idx] = labs(target_steps[idx]-position_steps[idx]);
      }
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] + target_steps[Y_AXIS]-position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else if (idx == B_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] - target_steps[Y_AXIS]+position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else {
        delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
      }
    #else
      target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
      block->steps[idx] = labs(target_steps[idx]-position_steps[idx]);
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
	  #endif
    unit_vec[idx] = delta_mm; // 存储单位向量的分子

    // 设置方向位。启用的位表示方向为负。
    if (delta_mm < 0.0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  }

  // 如果这是一个零长度块，则退出。极不可能发生。
  if (block->step_event_count == 0) { return(PLAN_EMPTY_BLOCK); }

  // 计算线性移动的单位向量以及块的最大进给速率和加速度，确保不超过各轴的最大值。
  // 注意：该计算假设所有轴都是正交的（笛卡尔坐标系），并且可以与 ABC 轴一起工作，
  // 如果它们也是正交/独立的。作用于单位向量的绝对值。
  block->millimeters = convert_delta_vector_to_unit_vector(unit_vec);
  block->acceleration = limit_value_by_axis_maximum(settings.acceleration, unit_vec);
  block->rapid_rate = limit_value_by_axis_maximum(settings.max_rate, unit_vec);

  // 存储编程速率。
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { block->programmed_rate = block->rapid_rate; }
  else { 
    block->programmed_rate = pl_data->feed_rate;
    if (block->condition & PL_COND_FLAG_INVERSE_TIME) { block->programmed_rate *= block->millimeters; }
  }

  // TODO: 需要检查在从静止状态开始时处理零连接速度的方法。
  if ((block_buffer_head == block_buffer_tail) || (block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {

    // 将块入口速度初始化为零。假设它将从静止开始。规划器将在稍后修正。
    // 如果是系统运动，则始终假设系统运动块从静止开始，并在完全停止时结束。
    block->entry_speed_sqr = 0.0;
    block->max_junction_speed_sqr = 0.0; // 从静止开始。强制从零速度开始。

  } else {
    // 通过向心加速度近似计算连接处的最大允许入口速度。
    // 让一个圆与之前和当前路径线段相切，连接处的偏差定义为连接处到圆的最近边缘的距离，
    // 与圆心共线。连接这两条路径的圆弧段表示向心加速度的路径。根据关于圆的半径的最大加速度求解最大速度，
    // 半径间接由连接处的偏差定义。这也可以被视为路径宽度或以前 Grbl 版本中的最大加速度。
    // 该方法实际上并没有偏离路径，但作为计算拐角速度的鲁棒方法，因为它考虑了连接角和连接速度的非线性。
    //
    // 注意：如果连接偏差值是有限的，Grbl 将以精确路径模式（G61）执行运动。
    // 如果连接偏差值为零，Grbl 将以精确停止模式（G61.1）执行运动。
    // 在未来，如果需要连续模式（G64），此处的数学运算完全相同。
    // 机器将不会运动到连接点，而是遵循这里定义的圆弧。Arduino 没有足够的 CPU 周期执行连续模式路径，
    // 但基于 ARM 的微控制器肯定可以。
    //
    // 注意：最大连接速度是固定值，因为机器加速度限制在操作期间无法动态更改，且线性移动几何形状也无法更改。
    // 在进给速率覆盖改变块的名义速度时，必须将其保存在内存中，这可能会改变所有块的整体最大入口速度条件。

    float junction_unit_vec[N_AXIS];
    float junction_cos_theta = 0.0;
    for (idx=0; idx<N_AXIS; idx++) {
      junction_cos_theta -= pl.previous_unit_vec[idx]*unit_vec[idx];
      junction_unit_vec[idx] = unit_vec[idx]-pl.previous_unit_vec[idx];
    }

    // 注意：通过三角函数半角恒等式计算，无需任何昂贵的三角函数 sin() 或 acos()。
    if (junction_cos_theta > 0.999999) {
      // 对于 0 度的锐角连接，只需设置最小连接速度。
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      if (junction_cos_theta < -0.999999) {
        // 连接是直线或 180 度。连接速度是无限的。
        block->max_junction_speed_sqr = SOME_LARGE_VALUE;
      } else {
        convert_delta_vector_to_unit_vector(junction_unit_vec);
        float junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
        float sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); // 三角半角恒等式。始终为正。
        block->max_junction_speed_sqr = max(MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                       (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2));
      }
    }
  }

  // 阻止系统运动更新此数据，以确保下一个 G-code 运动正确计算。
  if (!(block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {
    float nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);
    pl.previous_nominal_speed = nominal_speed;

    // 更新前一个路径单位向量和规划器位置。
    memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
    memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

    // 新块已设置。更新缓冲区头和下一个缓冲区头索引。
    block_buffer_head = next_buffer_head;
    next_buffer_head = plan_next_block_index(block_buffer_head);

    // 最后通过新块重新计算计划。
    planner_recalculate();
  }
  return(PLAN_OK);
}


// 重置规划器位置向量。由系统中止/初始化例程调用。
void plan_sync_position()
{
  // TODO：对于与机器位置不在同一坐标系中的电机配置，
  // 此函数需要更新以适应差异。
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
      if (idx==X_AXIS) {
        pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      } else if (idx==Y_AXIS) {
        pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      } else {
        pl.position[idx] = sys_position[idx];
      }
    #else
      pl.position[idx] = sys_position[idx];
    #endif
  }
}


// 返回规划器缓冲区中可用块的数量。
uint8_t plan_get_block_buffer_available()
{
  if (block_buffer_head >= block_buffer_tail) { return((BLOCK_BUFFER_SIZE-1)-(block_buffer_head-block_buffer_tail)); }
  return((block_buffer_tail-block_buffer_head-1));
}


// 返回规划器缓冲区中活动块的数量。
// 注意：已弃用。除非在 config.h 中启用经典状态报告，否则不使用。
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


// 使用假定存在于缓冲区尾部的部分完成的块重新初始化缓冲区计划。
// 在步进电机完全停止以进行进给保持并停止循环后调用。
void plan_cycle_reinitialize()
{
  // 从完全停止重新规划。重置规划器入口速度和缓冲计划指针。
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();
}
