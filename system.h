/*
  system.h - 系统级命令和实时处理的头文件
  Grbl的一部分

  版权所有 (c) 2014-2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改它，
  无论是许可证的第3版，还是（根据您的选择）任何更高版本。

  Grbl的发行是希望它会有用，但不提供任何担保；
  甚至没有适销性或特定用途适合性的暗示担保。有关更多详细信息，请参见
  GNU通用公共许可证。

  您应该已经收到了一份GNU通用公共许可证的副本
  与Grbl一起。如果没有，请访问<http://www.gnu.org/licenses/>。
*/

#ifndef system_h
#define system_h

#include "grbl.h"

// 定义系统执行器位图。由实时协议在内部使用，作为实时命令标志，
// 通知主程序异步执行指定的实时命令。
// 注意：系统执行器使用一个无符号的8位易失性变量（8个标志限制）。默认
// 标志始终为假，因此实时协议只需检查非零值即可
// 知道何时有实时命令需要执行。
#define EXEC_STATUS_REPORT  bit(0) // 位掩码 00000001
#define EXEC_CYCLE_START    bit(1) // 位掩码 00000010
#define EXEC_CYCLE_STOP     bit(2) // 位掩码 00000100
#define EXEC_FEED_HOLD      bit(3) // 位掩码 00001000
#define EXEC_RESET          bit(4) // 位掩码 00010000
#define EXEC_SAFETY_DOOR    bit(5) // 位掩码 00100000
#define EXEC_MOTION_CANCEL  bit(6) // 位掩码 01000000
#define EXEC_SLEEP          bit(7) // 位掩码 10000000

// 报警执行器代码。有效值（1-255）。零被保留。
#define EXEC_ALARM_HARD_LIMIT           1
#define EXEC_ALARM_SOFT_LIMIT           2
#define EXEC_ALARM_ABORT_CYCLE          3
#define EXEC_ALARM_PROBE_FAIL_INITIAL   4
#define EXEC_ALARM_PROBE_FAIL_CONTACT   5
#define EXEC_ALARM_HOMING_FAIL_RESET    6
#define EXEC_ALARM_HOMING_FAIL_DOOR     7
#define EXEC_ALARM_HOMING_FAIL_PULLOFF  8
#define EXEC_ALARM_HOMING_FAIL_APPROACH 9

// 覆盖位图。实时位标志控制进给、快速、主轴和冷却液覆盖。
// 主轴/冷却液和进给/快速被分为两个控制标志变量。
#define EXEC_FEED_OVR_RESET         bit(0)
#define EXEC_FEED_OVR_COARSE_PLUS   bit(1)
#define EXEC_FEED_OVR_COARSE_MINUS  bit(2)
#define EXEC_FEED_OVR_FINE_PLUS     bit(3)
#define EXEC_FEED_OVR_FINE_MINUS    bit(4)
#define EXEC_RAPID_OVR_RESET        bit(5)
#define EXEC_RAPID_OVR_MEDIUM       bit(6)
#define EXEC_RAPID_OVR_LOW          bit(7)
// #define EXEC_RAPID_OVR_EXTRA_LOW   bit(*) // *不支持*

#define EXEC_SPINDLE_OVR_RESET         bit(0)
#define EXEC_SPINDLE_OVR_COARSE_PLUS   bit(1)
#define EXEC_SPINDLE_OVR_COARSE_MINUS  bit(2)
#define EXEC_SPINDLE_OVR_FINE_PLUS     bit(3)
#define EXEC_SPINDLE_OVR_FINE_MINUS    bit(4)
#define EXEC_SPINDLE_OVR_STOP          bit(5)
#define EXEC_COOLANT_FLOOD_OVR_TOGGLE  bit(6)
#define EXEC_COOLANT_MIST_OVR_TOGGLE   bit(7)

// 定义系统状态位图。状态变量主要跟踪Grbl的各个功能，
// 以便管理每个功能而不发生重叠。它也用作
// 关键事件的消息标志。
#define STATE_IDLE          0      // 必须为零。没有标志。
#define STATE_ALARM         bit(0) // 在报警状态。锁定所有g-code进程。允许访问设置。
#define STATE_CHECK_MODE    bit(1) // G-code检查模式。只锁定计划和运动。
#define STATE_HOMING        bit(2) // 执行回零周期
#define STATE_CYCLE         bit(3) // 循环正在运行或正在执行运动。
#define STATE_HOLD          bit(4) // 活动进给保持
#define STATE_JOG           bit(5) // 手动操作模式。
#define STATE_SAFETY_DOOR   bit(6) // 安全门开启。进给保持并断电系统。
#define STATE_SLEEP         bit(7) // 睡眠状态。

// 定义系统挂起标志。用于以各种方式管理挂起状态和过程。
#define SUSPEND_DISABLE           0      // 必须为零。
#define SUSPEND_HOLD_COMPLETE     bit(0) // 指示初始进给保持已完成。
#define SUSPEND_RESTART_RETRACT   bit(1) // 指示从恢复停车运动的撤回标志。
#define SUSPEND_RETRACT_COMPLETE  bit(2) // （仅安全门）指示撤回和断电已完成。
#define SUSPEND_INITIATE_RESTORE  bit(3) // （仅安全门）指示从循环启动恢复程序的标志。
#define SUSPEND_RESTORE_COMPLETE  bit(4) // （仅安全门）指示准备恢复正常操作。
#define SUSPEND_SAFETY_DOOR_AJAR  bit(5) // 跟踪安全门状态以便恢复。
#define SUSPEND_MOTION_CANCEL     bit(6) // 指示取消恢复运动。当前由探测例程使用。
#define SUSPEND_JOG_CANCEL        bit(7) // 指示正在处理的手动操作取消，并在完成时重置缓冲区。

// 定义步进段生成器状态标志。
#define STEP_CONTROL_NORMAL_OP            0  // 必须为零。
#define STEP_CONTROL_END_MOTION           bit(0)
#define STEP_CONTROL_EXECUTE_HOLD         bit(1)
#define STEP_CONTROL_EXECUTE_SYS_MOTION   bit(2)
#define STEP_CONTROL_UPDATE_SPINDLE_PWM   bit(3)

// 定义Grbl内部使用的控制引脚索引。引脚映射可能会改变，但这些值不会。
#define N_CONTROL_PIN 4
#define CONTROL_PIN_INDEX_SAFETY_DOOR   bit(0)
#define CONTROL_PIN_INDEX_RESET         bit(1)
#define CONTROL_PIN_INDEX_FEED_HOLD     bit(2)
#define CONTROL_PIN_INDEX_CYCLE_START   bit(3)

// 定义主轴停止覆盖控制状态。
#define SPINDLE_STOP_OVR_DISABLED       0  // 必须为零。
#define SPINDLE_STOP_OVR_ENABLED        bit(0)
#define SPINDLE_STOP_OVR_INITIATE       bit(1)
#define SPINDLE_STOP_OVR_RESTORE        bit(2)
#define SPINDLE_STOP_OVR_RESTORE_CYCLE  bit(3)

// 定义全局系统变量
typedef struct {
  uint8_t state;               // 跟踪Grbl的当前系统状态。
  uint8_t abort;               // 系统中止标志。强制退出回主循环以进行重置。             
  uint8_t suspend;             // 系统挂起位标志变量，管理保持、取消和安全门。
  uint8_t soft_limit;          // 跟踪状态机的软限制错误。（布尔值）
  uint8_t step_control;        // 根据系统状态管理步进段生成器。
  uint8_t probe_succeeded;     // 跟踪最后一次探测周期是否成功。
  uint8_t homing_axis_lock;    // 当限位开关触发时锁定轴。用作步进器ISR中的轴运动掩码。
  uint8_t f_override;          // 进给率覆盖值（百分比）
  uint8_t r_override;          // 快速覆盖值（百分比）
  uint8_t spindle_speed_ovr;   // 主轴速度值（百分比）
  uint8_t spindle_stop_ovr;    // 跟踪主轴停止覆盖状态
  uint8_t report_ovr_counter;  // 跟踪何时将覆盖数据添加到状态报告。
  uint8_t report_wco_counter;  // 跟踪何时将工作坐标偏移数据添加到状态报告。
  float spindle_speed;
} system_t;
extern system_t sys;

// 注意：如果出现问题，这些位置变量可能需要声明为易失性。
int32_t sys_position[N_AXIS];      // 实时机器（即原点）位置向量，以步骤为单位。
int32_t sys_probe_position[N_AXIS]; // 上一次探测位置，以机器坐标和步骤为单位。

volatile uint8_t sys_probe_state;   // 探测状态值。用于协调探测周期与步进器ISR。
volatile uint8_t sys_rt_exec_state;   // 用于状态管理的全局实时执行器位标志变量。见EXEC位掩码。
volatile uint8_t sys_rt_exec_alarm;   // 用于设置各种警报的全局实时执行器位标志变量。
volatile uint8_t sys_rt_exec_motion_override; // 用于运动相关覆盖的全局实时执行器位标志变量。
volatile uint8_t sys_rt_exec_accessory_override; // 用于主轴/冷却液覆盖的全局实时执行器位标志变量。

#ifdef DEBUG
  #define EXEC_DEBUG_REPORT  bit(0)
  volatile uint8_t sys_rt_exec_debug;
#endif

// 初始化串行协议
void system_init();

// 返回控制引脚状态的位域，按CONTROL_PIN_INDEX组织。（1=触发，0=未触发）。
uint8_t system_control_get_state();

// 根据引脚状态返回安全门是开启还是关闭。
uint8_t system_check_safety_door_ajar();

// 执行内部系统命令，定义为以'$'开头的字符串
uint8_t system_execute_line(char *line);

// 在初始化时执行存储在EEPROM中的启动脚本行
void system_execute_startup(char *line);

void system_flag_wco_change();

// 返回轴'idx'的机器位置。必须发送一个'step'数组。
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx);

// 根据发送的'step'数组更新机器'position'数组。
void system_convert_array_steps_to_mpos(float *position, int32_t *steps);

// 仅限CoreXY计算。根据CoreXY电机步骤返回x或y轴的"steps"。
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps);
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps);
#endif

// 检查并报告目标数组是否超过机器行程限制。
uint8_t system_check_travel_limits(float *target);

// 用于设置和清除Grbl的实时执行标志的特殊处理程序。
void system_set_exec_state_flag(uint8_t mask);
void system_clear_exec_state_flag(uint8_t mask);
void system_set_exec_alarm(uint8_t code);
void system_clear_exec_alarm();
void system_set_exec_motion_override_flag(uint8_t mask);
void system_set_exec_accessory_override_flag(uint8_t mask);
void system_clear_exec_motion_overrides();
void system_clear_exec_accessory_overrides();

#endif
