/*
  probe.c - 与探测方法相关的代码
  Grbl的一部分

  版权所有 (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改
  它，许可证的版本为3，或（根据您的选择）任何更高版本。

  Grbl的分发是希望它会有用，
  但不提供任何担保；甚至没有对适销性或特定用途适用性的暗示担保。有关详细信息，请参见
  GNU通用公共许可证。

  您应该已经收到了GNU通用公共许可证的副本
  随Grbl一起。如果没有，请参见<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 根据用户设置和探测循环模式反转探针引脚状态。
uint8_t probe_invert_mask;

// 探针引脚初始化例程。
void probe_init()
{
  PROBE_DDR &= ~(PROBE_MASK); // 配置为输入引脚
#ifdef DISABLE_PROBE_PIN_PULL_UP
  PROBE_PORT &= ~(PROBE_MASK); // 正常低操作。需要外部下拉电阻。
#else
  PROBE_PORT |= PROBE_MASK; // 启用内部上拉电阻。正常高操作。
#endif
  probe_configure_invert_mask(false); // 初始化反转掩码。
}

// 由probe_init()和mc_probe()例程调用。设置探针引脚的反转掩码，以
// 根据正常高/正常低操作的设置和朝向工件/远离工件的探测循环模式
// 适当地设置引脚逻辑。
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // 初始化为零。
  if (bit_isfalse(settings.flags, BITFLAG_INVERT_PROBE_PIN))
  {
    probe_invert_mask ^= PROBE_MASK;
  }
  if (is_probe_away)
  {
    probe_invert_mask ^= PROBE_MASK;
  }
}

// 返回探针引脚状态。触发 = 真。由G代码解析器和探针状态监视器调用。
uint8_t probe_get_state() { return ((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }

// 监视探针引脚状态并在检测到时记录系统位置。由步进电机ISR每个ISR滴答调用。
// 注意：此函数必须非常高效，以免拖慢步进电机ISR。
void probe_state_monitor()
{
  if (probe_get_state())
  {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
