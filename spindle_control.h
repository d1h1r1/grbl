/*
  spindle_control.h - 主轴控制方法
  Grbl 的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以在 GNU 通用公共许可证的条款下重新分发和/或修改
  该许可证由自由软件基金会发布，版本 3，或
  （根据您的选择）任何更高版本。

  Grbl 的发行是希望它能对您有用，
  但不提供任何担保；甚至没有对
  适销性或特定用途的适用性的默示担保。有关更多细节，请参阅
  GNU 通用公共许可证。

  您应该已收到 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请查看 <http://www.gnu.org/licenses/>。
*/

#ifndef spindle_control_h
#define spindle_control_h

#define SPINDLE_NO_SYNC false // 不同步
#define SPINDLE_FORCE_SYNC true // 强制同步

#define SPINDLE_STATE_DISABLE  0  // 必须为零。
#define SPINDLE_STATE_CW       bit(0) // 顺时针
#define SPINDLE_STATE_CCW      bit(1) // 逆时针

// 初始化主轴引脚和硬件 PWM（如果启用）。
void spindle_init();

// 当设置主轴状态并需要缓冲同步时由 G 代码解析器调用。
// 如果启用了 PWM，则立即通过 PWM 设置主轴运行状态及其方向和转速。
// 在同步和停车运动/主轴停止恢复期间由 spindle_sync() 调用。

// 当设置主轴状态并需要缓冲同步时由 G 代码解析器调用。
void spindle_sync(uint8_t state, float rpm);

// 设置主轴运行状态，包括方向、启用状态和主轴 PWM。
void spindle_set_state(uint8_t state, float rpm); 

// 快速设置主轴 PWM 以供步进 ISR 使用。也由 spindle_set_state() 调用。
// 注意：Mega2560 的 PWM 寄存器是 16 位的。
void spindle_set_speed(uint16_t pwm_value);

// 计算 Mega2560 特定的 PWM 寄存器值，以便快速更新给定的 RPM。
uint16_t spindle_compute_pwm_value(float rpm);
  
// 停止和启动主轴例程。由所有主轴例程和步进 ISR 调用。
void spindle_stop();

#endif
