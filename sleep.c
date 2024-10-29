/*
  sleep.c - 确定并执行睡眠程序
  Grbl 的一部分
  
  版权所有 (c) 2016 Sungeun K. Jeon  

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

#include "grbl.h" 

#define SLEEP_SEC_PER_OVERFLOW (65535.0*64.0/F_CPU) // 基于 16 位定时器大小和预分频器的溢出秒数
#define SLEEP_COUNT_MAX (SLEEP_DURATION/SLEEP_SEC_PER_OVERFLOW) // 最大睡眠计数

volatile uint8_t sleep_counter; // 睡眠计数器


// 初始化睡眠计数器并启用定时器。
static void sleep_enable() { 
  sleep_counter = 0; // 重置睡眠计数器
  TCNT3 = 0;  // 重置定时器 3 计数寄存器
  TIMSK3 |= (1<<TOIE3); // 启用定时器 3 溢出中断
} 


// 禁用睡眠定时器。
static void sleep_disable() { TIMSK3 &= ~(1<<TOIE3); } // 禁用定时器溢出中断


// 睡眠定时器的初始化例程。
void sleep_init()
{
  // 配置定时器 3：睡眠计数器溢出中断
  // 注意：通过使用溢出中断，定时器在溢出时会自动重新加载。
  TCCR3B = 0; // 正常操作。溢出。
  TCCR3A = 0;
  TCCR3B = (TCCR3B & ~((1<<CS32) | (1<<CS31))) | (1<<CS30); // 停止定时器
  // TCCR3B |= (1<<CS32); // 启用 1/256 预分频器的定时器。最大约 4.4 分钟，使用 uint8 和 1.05 秒/滴
  // TCCR3B |= (1<<CS31); // 启用 1/8 预分频器的定时器。最大约 8.3 秒，使用 uint8 和 32.7 毫秒/滴
  TCCR3B |= (1<<CS31)|(1<<CS30); // 启用 1/64 预分频器的定时器。最大约 66.8 秒，使用 uint8 和 0.262 秒/滴
  // TCCR3B |= (1<<CS32)|(1<<CS30); // 启用 1/1024 预分频器的定时器。最大约 17.8 分钟，使用 uint8 和 4.19 秒/滴
  sleep_disable();
}


// 每次定时器溢出时增加睡眠计数器。
ISR(TIMER3_OVF_vect) { sleep_counter++; }


// 如果满足运行条件，则启动睡眠定时器。当计时结束时，执行睡眠模式。
static void sleep_execute()
{
  // 获取串口接收缓冲区中当前的字符数量。
  uint8_t rx_initial = serial_get_rx_buffer_count();

  // 启用睡眠计数器
  sleep_enable();

  do {          
    // 监视任何新的串口接收数据或外部事件（查询、按钮、警报）以退出。
    if ( (serial_get_rx_buffer_count() > rx_initial) || sys_rt_exec_state || sys_rt_exec_alarm ) {
      // 禁用睡眠定时器并返回正常操作。
      sleep_disable();  
      return;
    }
  } while(sleep_counter <= SLEEP_COUNT_MAX);
  
  // 如果到达此处，睡眠计数器已到期。执行睡眠程序。
  // 通知用户 Grbl 已超时并将进入停车模式。
  // 要退出睡眠，恢复或重置即可。无论哪种方式，作业将无法恢复。
  report_feedback_message(MESSAGE_SLEEP_MODE);
  system_set_exec_state_flag(EXEC_SLEEP);
}


// 检查睡眠的运行条件。如果满足，则启用睡眠倒计时并在计时结束时执行
// 睡眠模式。
// 注意：睡眠程序可能是阻塞的，因为 Grbl 不接收任何命令，也不移动。
// 因此，确保任何有效的运行状态在执行睡眠定时器时不会处于移动状态。
void sleep_check()
{
  // 睡眠执行功能仅在机器处于空闲或保持状态并且
  // 启用了任何电源组件时继续。
  // 注意：在重载或激光模式下，模态主轴和冷却状态不能得到保证。需要 
  // 直接监控和记录停车期间的运行状态以确保正常功能。
  if (gc_state.modal.spindle || gc_state.modal.coolant) {
    if (sys.state == STATE_IDLE) { 
      sleep_execute();
    } else if ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE)) {
      sleep_execute();
    } else if (sys.state == STATE_SAFETY_DOOR && (sys.suspend & SUSPEND_RETRACT_COMPLETE)) {
      sleep_execute();
    }
  }
}  
