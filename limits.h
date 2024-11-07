/*
  limits.h - 与限位开关和执行归位循环相关的代码
  Grbl 的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

extern parser_block_t gc_block;
parser_state_t gc_state;
// 初始化限位模块
void limits_init();

// 禁用硬限制。
void limits_disable();

// 返回限位状态，作为位域无符号整型变量。
uint8_t limits_get_state();

// 根据输入设置执行归位循环的一部分。
void limits_go_home(uint8_t cycle_mask);

// 检查软限制违规
void limits_soft_check(float *target);
