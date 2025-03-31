/*
  main.c - 一个支持 rs274/ngc (G-code) 的嵌入式 CNC 控制器
  是 Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon, Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以在自由软件基金会发布的 GNU 通用公共许可证的条款下重新发布和/或修改它，
  该许可证的版本可以是许可证的第 3 版，或者（由您选择）任何更高版本。

  Grbl 的发布是希望它能够有用，
  但没有任何保证；甚至没有适销性或特定用途适用性的隐含保证。有关更多详细信息，请参见
  GNU 通用公共许可证。

  您应该已经收到了 GNU 通用公共许可证的副本
  以及 Grbl。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 声明系统全局变量结构
system_t sys;

int main(void)
{
  // 在上电时初始化系统。
  serial_init();   // 设置串行波特率和中断
  serial2_init();
  settings_init(); // 从 EEPROM 加载 Grbl 设置
  stepper_init();  // 配置步进电机引脚和中断定时器
  system_init();   // 配置引脚引脚和引脚变更中断

  // memset(sys_position, 0, sizeof(sys_position)); // 清除机器位置。
  sei(); // 启用中断

// 初始化系统状态。
#ifdef FORCE_INITIALIZATION_ALARM
         // 在电源循环或硬重置时强制 Grbl 进入 ALARM 状态。
  sys.state = STATE_ALARM;
#else
  sys.state = STATE_IDLE;
#endif

// 检查上电并设置系统警报，如果启用了归位，则强制归位周期
// 通过设置 Grbl 的警报状态。警报锁定所有 G-code 命令，包括
// 启动脚本，但允许访问设置和内部命令。只有归位
// 周期 '$H' 或解除警报锁 '$X' 将禁用警报。
// 注意：启动脚本将在归位周期成功完成后运行，但
// 不会在解除警报锁后运行。防止运动启动块失控地撞击物体。
// 非常糟糕。
#ifdef HOMING_INIT_LOCK
  if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE))
  {
    sys.state = STATE_ALARM;
  }
#endif

  // Grbl 在上电或系统中止时的初始化循环。对于后者，所有进程
  // 将返回到此循环以进行干净的重新初始化。
  for (;;)
  {
    // 重置系统变量。
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // 清除系统结构变量。
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;                 // 设置为 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE;                // 设置为 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // 设置为 100%
    // memset(sys_probe_position, 0, sizeof(sys_probe_position)); // 清除探测位置。
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // 重置 Grbl 主要系统。
    serial_reset_read_buffer(); // 清除串行读取缓冲区
    gc_init();                  // 将 G-code 解析器设置为默认状态
    spindle_init();
    coolant_init();
    limits_init();
    probe_control_init();
    // laser_control_init();
    tool_control_init();
    probe_init();
    sleep_init();
    plan_reset(); // 清除块缓冲区和规划器变量
    st_reset();   // 清除步进电机子系统变量。
    DDRH |= (1 << 0); // 将其配置为输出引脚。
    PORTH |= (1<<0);  // 设置引脚为高，继电器默认闭合

    // 将清除的 G-code 和规划器位置同步到当前系统位置。
    plan_sync_position();
    gc_sync_position();
    // 打印欢迎消息。指示在上电或重置时发生了初始化。
    report_init_message();

    // 启动 Grbl 主循环。处理程序输入并执行它们。
    protocol_main_loop();
  }
  return 0; /* 永远不会到达 */
}
