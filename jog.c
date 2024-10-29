/*
  jog.h - 移动方法
  Grbl 的一部分

  版权所有 (c) 2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl 是自由软件：你可以在自由软件基金会发布的 GNU 通用公共许可证条款下重新分发和/或修改
  它，许可证版本为 3，或（根据你的选择）任何更高版本。

  Grbl 的发布是为了希望它能有用，
  但不提供任何担保；甚至没有关于
  适销性或适用于特定目的的隐含担保。有关详细信息，请参见
  GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


// 设置从 G-code 解析器接收到的有效移动命令，检查软限位，并执行移动。
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block)
{
  // 初始化用于移动命令的规划数据结构。
  // 注意：在移动期间允许主轴和冷却液完全功能化并进行覆盖。
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  pl_data->line_number = gc_block->values.n;

  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (system_check_travel_limits(gc_block->values.xyz)) { return(STATUS_TRAVEL_EXCEEDED); }
  }

  // 有效的移动命令。计划、设置状态并执行。
  mc_line(gc_block->values.xyz,pl_data);
  if (sys.state == STATE_IDLE) {
    if (plan_get_current_block() != NULL) { // 检查是否有待执行的命令块。
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up();  // 注意：手动启动。不需要状态机。
    }
  }

  return(STATUS_OK);
}
