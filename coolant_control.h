/*
  coolant_control.h - 主轴控制方法
  Grbl的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl是一个自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款，重新发布和/或修改
  该软件，版本3或（根据您的选择）任何更新的版本。

  Grbl的发布是希望它能有用，
  但没有任何保证；甚至不含有
  适销性或特定用途的隐含保证。有关更多详细信息，请参见GNU通用公共许可证。

  您应该已随Grbl一起收到GNU通用公共许可证的副本。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/


#ifndef coolant_control_h
#define coolant_control_h

#define COOLANT_NO_SYNC     false
#define COOLANT_FORCE_SYNC  true

#define COOLANT_STATE_DISABLE   0  // 必须为零
#define COOLANT_STATE_FLOOD     bit(0)
#define COOLANT_STATE_MIST      bit(1)


// 初始化冷却液控制引脚。
void coolant_init();

// 返回当前冷却液输出状态。覆盖可能会改变其编程状态。
uint8_t coolant_get_state();

// 立即禁用冷却液引脚。
void coolant_stop();

// 根据指定的状态设置冷却液引脚。
void coolant_set_state(uint8_t mode);

// G-code解析器设置冷却状态的入口点。检查并执行额外条件。
void coolant_sync(uint8_t mode);

#endif
