/* 
  coolant_control.c - 冷却液控制方法
  Grbl的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改它，
  许可证的版本为第3版，或（根据您的选择）任何更高版本。

  Grbl的发布是希望它会有用，
  但不提供任何担保；甚至不包括适销性或特定目的适用性的隐含担保。有关更多详细信息，请参见
  GNU通用公共许可证。

  您应该在获取Grbl时收到一份GNU通用公共许可证的副本。如果没有，请访问<http://www.gnu.org/licenses/>。
*/


#include "grbl.h"


void coolant_init()
{
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT); // 将其配置为输出引脚。
  COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT); // 将其配置为输出引脚。
  coolant_stop();
}


// 返回当前冷却液输出状态。覆盖可能会从编程状态更改它。
uint8_t coolant_get_state()
{
  uint8_t cl_state = COOLANT_STATE_DISABLE;
  #ifdef INVERT_COOLANT_FLOOD_PIN
    if (bit_isfalse(COOLANT_FLOOD_PORT,(1 << COOLANT_FLOOD_BIT))) {
  #else
    if (bit_istrue(COOLANT_FLOOD_PORT,(1 << COOLANT_FLOOD_BIT))) {
  #endif
    cl_state |= COOLANT_STATE_FLOOD;
  }
  #ifdef INVERT_COOLANT_MIST_PIN
    if (bit_isfalse(COOLANT_MIST_PORT,(1 << COOLANT_MIST_BIT))) {
  #else
    if (bit_istrue(COOLANT_MIST_PORT,(1 << COOLANT_MIST_BIT))) {
  #endif
    cl_state |= COOLANT_STATE_MIST;
  }
  return(cl_state);
}


// 直接由 coolant_init()、coolant_set_state() 和 mc_reset() 调用，这些调用可能发生在中断级别。没有设置报告标志，但仅由不需要它的例程调用。
void coolant_stop()
{
  #ifdef INVERT_COOLANT_FLOOD_PIN
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
  #else
    COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #endif
  #ifdef INVERT_COOLANT_MIST_PIN
    COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
  #else
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
  #endif
}


// 仅主程序。立即设置冷却液（flood coolant）运行状态，如果启用了雾状冷却（mist coolant），也会同时设置。还设置了一个标志，用于报告冷却状态的更新。
// 由冷却液开关覆盖、停车恢复、停车收回、睡眠模式、g-code解析程序结束和g-code解析程序coolant_sync()调用。
void coolant_set_state(uint8_t mode)
{
  if (sys.abort) { return; } // 中止期间阻塞。
  
  if (mode == COOLANT_DISABLE) {
  
    coolant_stop(); 
  
  } else {
  
    if (mode & COOLANT_FLOOD_ENABLE) {
      #ifdef INVERT_COOLANT_FLOOD_PIN
        COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
      #else
        COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
      #endif
    }
  
    if (mode & COOLANT_MIST_ENABLE) {
      #ifdef INVERT_COOLANT_MIST_PIN
        COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
      #else
        COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
      #endif
    }
  
  }
  sys.report_ovr_counter = 0; // 设置为立即报告更改
}



// G-code解析器设置冷却液状态的入口点。强制执行规划器缓冲区同步，如果中止或检查模式处于激活状态，则退出。
void coolant_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // 确保在程序中指定时开启冷却液。
  coolant_set_state(mode);
}
