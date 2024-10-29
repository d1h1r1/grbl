/*
  report.h - 报告和消息方法
  Grbl 的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl 是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证进行再发行和/或修改，
  该许可证的版本为 3，或（根据您的选择）任何后续版本。

  Grbl 的分发是出于它会有用的希望，
  但没有任何担保；甚至没有适销性或适合特定目的的隐含担保。有关更多详细信息，请参见
  GNU 通用公共许可证。

  您应该已收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/
#ifndef report_h
#define report_h

// 定义 Grbl 状态码。有效值（0-255）
#define STATUS_OK 0
#define STATUS_EXPECTED_COMMAND_LETTER 1
#define STATUS_BAD_NUMBER_FORMAT 2
#define STATUS_INVALID_STATEMENT 3
#define STATUS_NEGATIVE_VALUE 4
#define STATUS_SETTING_DISABLED 5
#define STATUS_SETTING_STEP_PULSE_MIN 6
#define STATUS_SETTING_READ_FAIL 7
#define STATUS_IDLE_ERROR 8
#define STATUS_SYSTEM_GC_LOCK 9
#define STATUS_SOFT_LIMIT_ERROR 10
#define STATUS_OVERFLOW 11
#define STATUS_MAX_STEP_RATE_EXCEEDED 12
#define STATUS_CHECK_DOOR 13
#define STATUS_LINE_LENGTH_EXCEEDED 14
#define STATUS_TRAVEL_EXCEEDED 15
#define STATUS_INVALID_JOG_COMMAND 16

#define STATUS_GCODE_UNSUPPORTED_COMMAND 20
#define STATUS_GCODE_MODAL_GROUP_VIOLATION 21
#define STATUS_GCODE_UNDEFINED_FEED_RATE 22
#define STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER 23
#define STATUS_GCODE_AXIS_COMMAND_CONFLICT 24
#define STATUS_GCODE_WORD_REPEATED 25
#define STATUS_GCODE_NO_AXIS_WORDS 26
#define STATUS_GCODE_INVALID_LINE_NUMBER 27
#define STATUS_GCODE_VALUE_WORD_MISSING 28
#define STATUS_GCODE_UNSUPPORTED_COORD_SYS 29
#define STATUS_GCODE_G53_INVALID_MOTION_MODE 30
#define STATUS_GCODE_AXIS_WORDS_EXIST 31
#define STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE 32
#define STATUS_GCODE_INVALID_TARGET 33
#define STATUS_GCODE_ARC_RADIUS_ERROR 34
#define STATUS_GCODE_NO_OFFSETS_IN_PLANE 35
#define STATUS_GCODE_UNUSED_WORDS 36
#define STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR 37
#define STATUS_GCODE_MAX_VALUE_EXCEEDED 38

// 定义 Grbl 报警代码。有效值（1-255）。0 为保留。
#define ALARM_HARD_LIMIT_ERROR      EXEC_ALARM_HARD_LIMIT
#define ALARM_SOFT_LIMIT_ERROR      EXEC_ALARM_SOFT_LIMIT
#define ALARM_ABORT_CYCLE           EXEC_ALARM_ABORT_CYCLE
#define ALARM_PROBE_FAIL_INITIAL    EXEC_ALARM_PROBE_FAIL_INITIAL
#define ALARM_PROBE_FAIL_CONTACT    EXEC_ALARM_PROBE_FAIL_CONTACT
#define ALARM_HOMING_FAIL_RESET     EXEC_ALARM_HOMING_FAIL_RESET
#define ALARM_HOMING_FAIL_DOOR      EXEC_ALARM_HOMING_FAIL_DOOR
#define ALARM_HOMING_FAIL_PULLOFF   EXEC_ALARM_HOMING_FAIL_PULLOFF
#define ALARM_HOMING_FAIL_APPROACH  EXEC_ALARM_HOMING_FAIL_APPROACH

// 定义 Grbl 反馈消息代码。有效值（0-255）。
#define MESSAGE_CRITICAL_EVENT 1
#define MESSAGE_ALARM_LOCK 2
#define MESSAGE_ALARM_UNLOCK 3
#define MESSAGE_ENABLED 4
#define MESSAGE_DISABLED 5
#define MESSAGE_SAFETY_DOOR_AJAR 6
#define MESSAGE_CHECK_LIMITS 7
#define MESSAGE_PROGRAM_END 8
#define MESSAGE_RESTORE_DEFAULTS 9
#define MESSAGE_SPINDLE_RESTORE 10
#define MESSAGE_SLEEP_MODE 11

// 打印系统状态消息。
void report_status_message(uint8_t status_code);

// 打印系统报警消息。
void report_alarm_message(uint8_t alarm_code);

// 打印杂项反馈消息。
void report_feedback_message(uint8_t message_code);

// 打印欢迎消息
void report_init_message();

// 打印 Grbl 帮助和当前全局设置
void report_grbl_help();

// 打印 Grbl 全局设置
void report_grbl_settings();

// 打印在执行前接收到的预解析行的回显。
void report_echo_line_received(char *line);

// 打印实时状态报告
void report_realtime_status();

// 打印记录的探测位置
void report_probe_parameters();

// 打印 Grbl NGC 参数（坐标偏移、探测）
void report_ngc_parameters();

// 打印当前的 G 代码解析器模式状态
void report_gcode_modes();

// 打印启动行（在请求和执行时）。
void report_startup_line(uint8_t n, char *line);
void report_execute_startup_message(char *line, uint8_t status_code);

// 打印构建信息和用户信息
void report_build_info(char *line);

#ifdef DEBUG
  void report_realtime_debug();
#endif

#endif
