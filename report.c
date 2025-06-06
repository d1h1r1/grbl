/*
  report.c - 报告和消息方法
  Grbl 的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

/*
  该文件作为 Grbl 的主要反馈接口。所有输出数据，例如协议状态消息、反馈消息和状态报告，都存储在这里。
  大部分情况下，这些函数主要从 protocol.c 方法中调用。如果需要不同风格的反馈（即 JSON），用户可以更改以下方法以满足需求。
*/

#include "grbl.h"

// 内部报告工具，用于将重复的任务转换为函数以减少闪存使用。
void report_util_setting_prefix(uint8_t n)
{
  serial_write('$');
  print_uint8_base10(n);
  serial_write('=');
}
static void report_util_line_feed() { printPgmString(PSTR("\r\n")); }
static void report_util_feedback_line_feed()
{
  serial_write(']');
  report_util_line_feed();
}
static void report_util_gcode_modes_G() { printPgmString(PSTR(" G")); }
static void report_util_gcode_modes_M() { printPgmString(PSTR(" M")); }
// static void report_util_comment_line_feed() { serial_write(')'); report_util_line_feed(); }
static void report_util_axis_values(float *axis_value)
{
  uint8_t idx;
  for (idx = 0; idx < 4; idx++)
  {
    printFloat_CoordValue(axis_value[idx]);
    if (idx < (4 - 1))
    {
      serial_write(',');
    }
  }
}

/*
static void report_util_setting_string(uint8_t n) {
  serial_write(' ');
  serial_write('(');
  switch(n) {
    case 0: printPgmString(PSTR("脉冲设置")); break;
    case 1: printPgmString(PSTR("空闲延迟")); break;
    case 2: printPgmString(PSTR("脉冲反转")); break;
    case 3: printPgmString(PSTR("方向反转")); break;
    case 4: printPgmString(PSTR("使能反转")); break;
    case 5: printPgmString(PSTR("限位反转")); break;
    case 6: printPgmString(PSTR("探针反转")); break;
    case 10: printPgmString(PSTR("重复")); break;
    case 11: printPgmString(PSTR("连接设备")); break;
    case 12: printPgmString(PSTR("弧容差")); break;
    case 13: printPgmString(PSTR("重复英寸")); break;
    case 20: printPgmString(PSTR("软限制")); break;
    case 21: printPgmString(PSTR("硬限制")); break;
    case 22: printPgmString(PSTR("归位循环")); break;
    case 23: printPgmString(PSTR("归位方向反转")); break;
    case 24: printPgmString(PSTR("归位进给")); break;
    case 25: printPgmString(PSTR("归位寻位")); break;
    case 26: printPgmString(PSTR("归位延迟")); break;
    case 27: printPgmString(PSTR("归位脉冲关闭")); break;
    case 30: printPgmString(PSTR("最大转速")); break;
    case 31: printPgmString(PSTR("最小转速")); break;
    case 32: printPgmString(PSTR("激光")); break;
    default:
      n -= AXIS_SETTINGS_START_VAL;
      uint8_t idx = 0;
      while (n >= AXIS_SETTINGS_INCREMENT) {
        n -= AXIS_SETTINGS_INCREMENT;
        idx++;
      }
      serial_write(n+'x');
      switch (idx) {
        case 0: printPgmString(PSTR(":步/mm")); break;
        case 1: printPgmString(PSTR(":mm/min")); break;
        case 2: printPgmString(PSTR(":mm/s^2")); break;
        case 3: printPgmString(PSTR(":mm 最大")); break;
      }
      break;
  }
  report_util_comment_line_feed();
}
*/

static void report_util_uint8_setting(uint8_t n, int val)
{
  report_util_setting_prefix(n);
  print_uint8_base10(val);
  report_util_line_feed(); // report_util_setting_string(n);
}
static void report_util_float_setting(uint8_t n, float val, uint8_t n_decimal)
{
  report_util_setting_prefix(n);
  printFloat(val, n_decimal);
  report_util_line_feed(); // report_util_setting_string(n);
}

// 处理主要的确认协议响应，用于流式接口和人类反馈。
// 对于每一行输入，该方法会在成功命令时响应 "ok"，或在命令或关键系统错误发生时响应 "error:"。
// 错误事件可以来自 G-code 解析器、设置模块或异步的关键错误，例如触发的硬限制。接口应始终监视这些响应。
void report_status_message(uint8_t status_code)
{
  switch (status_code)
  {
  case STATUS_OK: // STATUS_OK
    printPgmString(PSTR("ok\r\n"));
    break;
  default:
    printPgmString(PSTR("error:"));
    print_uint8_base10(status_code);
    report_util_line_feed();
  }
}

// 打印警报消息。
void report_alarm_message(uint8_t alarm_code)
{
  printPgmString(PSTR("警报:"));
  print_uint8_base10(alarm_code);
  report_util_line_feed();
  delay_ms(500); // 强制延迟以确保消息清除串行写入缓冲区。
}

// 打印反馈消息。此方法作为集中方式提供额外的用户反馈，针对非状态/警报消息协议的内容。这些消息包括设置警告、开关切换和如何退出警报。
// 注意：对于接口，消息总是放在括号内。如果安装了静音模式，消息编号代码小于零。
void report_feedback_message(uint8_t message_code)
{
  printPgmString(PSTR("[消息:"));
  switch (message_code)
  {
  case MESSAGE_CRITICAL_EVENT:
    printPgmString(PSTR("重置以继续"));
    break;
  case MESSAGE_ALARM_LOCK:
    printPgmString(PSTR("'$H'|'$X' 解锁"));
    break;
  case MESSAGE_ALARM_UNLOCK:
    printPgmString(PSTR("警告：已解锁"));
    break;
  case MESSAGE_ENABLED:
    printPgmString(PSTR("已启用"));
    break;
  case MESSAGE_DISABLED:
    printPgmString(PSTR("已禁用"));
    break;
  case MESSAGE_SAFETY_DOOR_AJAR:
    printPgmString(PSTR("检查门"));
    break;
  case MESSAGE_CHECK_LIMITS:
    printPgmString(PSTR("检查限制"));
    break;
  case MESSAGE_PROGRAM_END:
    printPgmString(PSTR("程序结束"));
    break;
  case MESSAGE_RESTORE_DEFAULTS:
    printPgmString(PSTR("正在恢复默认值"));
    break;
  case MESSAGE_SPINDLE_RESTORE:
    printPgmString(PSTR("正在恢复主轴"));
    break;
  case MESSAGE_SLEEP_MODE:
    printPgmString(PSTR("睡眠中"));
    break;
  }
  report_util_feedback_line_feed();
}

// 欢迎消息
void report_init_message()
{
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION " ['$' 获取帮助]\r\n"));
}

// Grbl 帮助消息
void report_grbl_help()
{
  printPgmString(PSTR("[帮助:$$ $# $G $I $N $x=值 $Nx=行 $J=行 $SLP $C $X $H ~ ! ? ctrl-x]\r\n"));
}

// Grbl 全局设置打印。
// 注意：这里的编号方案必须与 settings.c 中的存储相关。
void report_grbl_settings()
{
  // 打印 Grbl 设置。
  report_util_uint8_setting(0, settings.pulse_microseconds);
  report_util_uint8_setting(1, settings.stepper_idle_lock_time);
  report_util_uint8_setting(2, settings.step_invert_mask);
  report_util_uint8_setting(3, settings.dir_invert_mask);
  report_util_uint8_setting(4, bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE));
  report_util_uint8_setting(5, bit_istrue(settings.flags, BITFLAG_INVERT_LIMIT_PINS));
  report_util_uint8_setting(6, bit_istrue(settings.flags, BITFLAG_INVERT_PROBE_PIN));
  report_util_uint8_setting(10, settings.status_report_mask);
  report_util_float_setting(11, settings.junction_deviation, N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(12, settings.arc_tolerance, N_DECIMAL_SETTINGVALUE);
  report_util_uint8_setting(13, bit_istrue(settings.flags, BITFLAG_REPORT_INCHES));
  report_util_uint8_setting(20, bit_istrue(settings.flags, BITFLAG_SOFT_LIMIT_ENABLE));
  report_util_uint8_setting(21, bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE));
  report_util_uint8_setting(22, bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE));
  report_util_uint8_setting(23, settings.homing_dir_mask);
  report_util_float_setting(24, settings.homing_feed_rate, N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(25, settings.homing_seek_rate, N_DECIMAL_SETTINGVALUE);
  report_util_uint8_setting(26, settings.homing_debounce_delay);
  report_util_float_setting(27, settings.homing_pulloff, N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(30, settings.rpm_max, N_DECIMAL_RPMVALUE);
  report_util_float_setting(31, settings.rpm_min, N_DECIMAL_RPMVALUE);
  report_util_uint8_setting(32, bit_istrue(settings.flags, BITFLAG_LASER_MODE));
  // 打印轴设置
  uint8_t idx, set_idx, tool_number;
  uint8_t val = AXIS_SETTINGS_START_VAL;
  for (set_idx = 0; set_idx < AXIS_N_SETTINGS; set_idx++)
  {
    for (idx = 0; idx < N_AXIS; idx++)
    {
      switch (set_idx)
      {
      case 0:
        report_util_float_setting(val + idx, settings.steps_per_mm[idx], N_DECIMAL_SETTINGVALUE);
        break;
      case 1:
        report_util_float_setting(val + idx, settings.max_rate[idx], N_DECIMAL_SETTINGVALUE);
        break;
      case 2:
        report_util_float_setting(val + idx, settings.acceleration[idx] / (60 * 60), N_DECIMAL_SETTINGVALUE);
        break;
      case 3:
        report_util_float_setting(val + idx, -settings.max_travel[idx], N_DECIMAL_SETTINGVALUE);
        break;
      }
    }
    val += AXIS_SETTINGS_INCREMENT;
  }
  uint8_t tool_val = TOOL_SETTINGS_START_VAL;
  report_util_uint8_setting(200, settings.tool);
  report_util_float_setting(201, settings.tool_length, N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(202, settings.tool_zpos, N_DECIMAL_SETTINGVALUE);
  for (tool_number = 0; tool_number < TOOL_NUM; tool_number++)
  {
    for (idx = 0; idx < 3; idx++)
    {
      switch (idx)
      {
      case 0:
        report_util_float_setting(tool_val + idx, settings.tool_x[tool_number], N_DECIMAL_SETTINGVALUE);
        break;
      case 1:
        report_util_float_setting(tool_val + idx, settings.tool_y[tool_number], N_DECIMAL_SETTINGVALUE);
        break;
      case 2:
        report_util_float_setting(tool_val + idx, settings.tool_z[tool_number], N_DECIMAL_SETTINGVALUE);
        break;
      }
    }
    tool_val += 3;
  }
}

void report_tool()
{
  uint8_t idx, tool_number, tool_val = TOOL_SETTINGS_START_VAL;
  for (tool_number = 0; tool_number < TOOL_NUM; tool_number++)
  {
    for (idx = 0; idx < 3; idx++)
    {
      switch (idx)
      {
      case 0:
        report_util_float_setting(tool_val + idx, settings.tool_x[tool_number], N_DECIMAL_SETTINGVALUE);
        break;
      case 1:
        report_util_float_setting(tool_val + idx, settings.tool_y[tool_number], N_DECIMAL_SETTINGVALUE);
        break;
      case 2:
        report_util_float_setting(tool_val + idx, settings.tool_z[tool_number], N_DECIMAL_SETTINGVALUE);
        break;
      }
    }
    tool_val += 3;
  }
}
// 打印当前探针参数。在探针命令时，这些参数会在成功探测或在 G38.3 成功探测命令时更新（如果支持）。
// 这些值在 Grbl 断电之前保持，断电后将重置。
void report_probe_parameters()
{
  // 以机器位置的方式报告。
  printPgmString(PSTR("[探针:"));
  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position, sys_probe_position);
  report_util_axis_values(print_position);
  serial_write(':');
  print_uint8_base10(sys.probe_succeeded);
  report_util_feedback_line_feed();
}

// 打印 Grbl NGC 参数（坐标偏移、探测）
void report_ngc_parameters()
{
  float coord_data[N_AXIS];
  uint8_t coord_select;
  for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++)
  {
    if (!(settings_read_coord_data(coord_select, coord_data)))
    {
      report_status_message(STATUS_SETTING_READ_FAIL);
      return;
    }
    printPgmString(PSTR("[G"));
    switch (coord_select)
    {
    case 6:
      printPgmString(PSTR("28"));
      break;
    case 7:
      printPgmString(PSTR("30"));
      break;
    default:
      print_uint8_base10(coord_select + 54);
      break; // G54-G59
    }
    serial_write(':');
    report_util_axis_values(coord_data);
    report_util_feedback_line_feed();
  }
  printPgmString(PSTR("[G92:")); // 打印 G92,G92.1，这些值在内存中不持久
  report_util_axis_values(gc_state.coord_offset);
  report_util_feedback_line_feed();
  printPgmString(PSTR("[TLO:")); // 打印工具长度偏移值
  printFloat_CoordValue(gc_state.tool_length_offset);
  report_util_feedback_line_feed();
  printPgmString(PSTR("[Zpos:")); // 打印z轴位置
  printFloat_CoordValue(settings.tool_zpos);
  report_util_feedback_line_feed();
  report_probe_parameters(); // 打印探针参数。内存中不持久。
}

// 打印当前的 G-code 解析器模式状态
void report_gcode_modes()
{
  printPgmString(PSTR("[GC:G"));
  if (gc_state.modal.motion >= MOTION_MODE_PROBE_TOWARD)
  {
    printPgmString(PSTR("38."));
    print_uint8_base10(gc_state.modal.motion - (MOTION_MODE_PROBE_TOWARD - 2));
  }
  else
  {
    print_uint8_base10(gc_state.modal.motion);
  }

  report_util_gcode_modes_G();
  print_uint8_base10(gc_state.modal.coord_select + 54);

  report_util_gcode_modes_G();
  print_uint8_base10(gc_state.modal.plane_select + 17);

  report_util_gcode_modes_G();
  print_uint8_base10(21 - gc_state.modal.units);

  report_util_gcode_modes_G();
  print_uint8_base10(gc_state.modal.distance + 90);

  report_util_gcode_modes_G();
  print_uint8_base10(94 - gc_state.modal.feed_rate);

  if (gc_state.modal.program_flow)
  {
    report_util_gcode_modes_M();
    switch (gc_state.modal.program_flow)
    {
    case PROGRAM_FLOW_PAUSED:
      serial_write('0');
      break;
    // case PROGRAM_FLOW_OPTIONAL_STOP: serial_write('1'); break; // M1 被忽略且不支持。
    case PROGRAM_FLOW_COMPLETED_M2:
    case PROGRAM_FLOW_COMPLETED_M30:
      print_uint8_base10(gc_state.modal.program_flow);
      break;
    }
  }

  report_util_gcode_modes_M();
  switch (gc_state.modal.spindle)
  {
  case SPINDLE_ENABLE_CW:
    serial_write('3');
    break;
  case SPINDLE_ENABLE_CCW:
    serial_write('4');
    break;
  case SPINDLE_DISABLE:
    serial_write('5');
    break;
  }

  report_util_gcode_modes_M();
  if (gc_state.modal.coolant)
  { // 注意：多个冷却状态可能同时处于激活状态。
    if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_MIST)
    {
      serial_write('7');
    }
    if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_FLOOD)
    {
      serial_write('8');
    }
  }
  else
  {
    serial_write('9');
  }

  printPgmString(PSTR(" T"));
  print_uint8_base10(settings.tool);

  printPgmString(PSTR(" F"));
  printFloat_RateValue(gc_state.feed_rate);

  printPgmString(PSTR(" S"));
  printFloat(gc_state.spindle_speed, N_DECIMAL_RPMVALUE);

  report_util_feedback_line_feed();
}

// 打印指定的启动行
void report_startup_line(uint8_t n, char *line)
{
  printPgmString(PSTR("$N"));
  print_uint8_base10(n);
  serial_write('=');
  printString(line);
  report_util_line_feed();
}

void report_execute_startup_message(char *line, uint8_t status_code)
{
  serial_write('>');
  printString(line);
  serial_write(':');
  report_status_message(status_code);
}

// 打印构建信息行
void report_build_info(char *line)
{
  printPgmString(PSTR("[VER:" GRBL_VERSION "." GRBL_VERSION_BUILD ":"));
  printString(line);
  report_util_feedback_line_feed();
  printPgmString(PSTR("[OPT:")); // 生成编译时构建选项列表
  serial_write('V');
  serial_write('N');
  serial_write('M');
#ifdef COREXY
  serial_write('C');
#endif
#ifdef PARKING_ENABLE
  serial_write('P');
#endif
#ifdef HOMING_FORCE_SET_ORIGIN
  serial_write('Z');
#endif
#ifdef HOMING_SINGLE_AXIS_COMMANDS
  serial_write('H');
#endif
#ifdef LIMITS_TWO_SWITCHES_ON_AXES
  serial_write('L');
#endif
#ifdef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
  serial_write('A');
#endif
#ifndef ENABLE_RESTORE_EEPROM_WIPE_ALL // 注意：当禁用时显示。
  serial_write('*');
#endif
#ifndef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // 注意：当禁用时显示。
  serial_write('$');
#endif
#ifndef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // 注意：当禁用时显示。
  serial_write('#');
#endif
#ifndef ENABLE_BUILD_INFO_WRITE_COMMAND // 注意：当禁用时显示。
  serial_write('I');
#endif
#ifndef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE // 注意：当禁用时显示。
  serial_write('E');
#endif
#ifndef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // 注意：当禁用时显示。
  serial_write('W');
#endif
  // 注意：编译的值，如覆盖增量/最大/最小值，可能在某个时候被添加。
  // 这些可能有逗号分隔符来分隔它们。

  report_util_feedback_line_feed();
}

// 打印 Grbl 从用户接收到的字符字符串行，该行已预解析，
// 并已发送到 protocol_execute_line() 例程以由 Grbl 执行。
void report_echo_line_received(char *line)
{
  printPgmString(PSTR("[echo: "));
  printString(line);
  report_util_feedback_line_feed();
}

// 打印实时数据。此功能获取步进子程序的实时快照
// 和 CNC 机器的实际位置。用户可以根据自己的特定需求更改以下函数，
// 但所需的实时数据报告必须尽可能简短。这是必需的，因为它最小化了计算开销，
// 使 Grbl 能够平稳运行，特别是在具有快速、短行段和高频报告（5-20Hz）
// 的 G-code 程序期间。
void report_realtime_status()
{
  uint8_t idx;
  int32_t current_position[N_AXIS]; // 复制系统位置变量的当前状态
  memcpy(current_position, sys_position, sizeof(sys_position));
  float print_position[N_AXIS];
  float offset_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position, current_position);
  system_convert_array_steps_to_mpos(offset_position, current_position);

  // 报告当前机器状态和子状态
  serial_write('<');
  switch (sys.state)
  {
  case STATE_IDLE:
    printPgmString(PSTR("Idle"));
    break;
  case STATE_CYCLE:
    printPgmString(PSTR("Run"));
    break;
  case STATE_HOLD:
    if (!(sys.suspend & SUSPEND_JOG_CANCEL))
    {
      printPgmString(PSTR("Hold:"));
      if (sys.suspend & SUSPEND_HOLD_COMPLETE)
      {
        serial_write('0');
      } // 准备恢复
      else
      {
        serial_write('1');
      } // 正在保持
      break;
    } // 在取消 jog 时继续打印 jog 状态。
  case STATE_JOG:
    printPgmString(PSTR("Jog"));
    break;
  case STATE_HOMING:
    printPgmString(PSTR("Home"));
    break;
  case STATE_ALARM:
    printPgmString(PSTR("Alarm"));
    break;
  case STATE_CHECK_MODE:
    printPgmString(PSTR("Check"));
    break;
  case STATE_SAFETY_DOOR:
    printPgmString(PSTR("Door:"));
    if (sys.suspend & SUSPEND_INITIATE_RESTORE)
    {
      serial_write('3'); // 正在恢复
    }
    else
    {
      if (sys.suspend & SUSPEND_RETRACT_COMPLETE)
      {
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)
        {
          serial_write('1'); // 门未关好
        }
        else
        {
          serial_write('0');
        } // 门已关闭并准备恢复
      }
      else
      {
        serial_write('2'); // 正在收回
      }
    }
    break;
  case STATE_SLEEP:
    printPgmString(PSTR("Sleep"));
    break;
  }

  float wco[N_AXIS];
  if (bit_isfalse(settings.status_report_mask, BITFLAG_RT_STATUS_POSITION_TYPE) ||
      (sys.report_wco_counter == 0))
  {
    for (idx = 0; idx < N_AXIS; idx++)
    {
      // 将工件坐标偏移和刀具长度偏移应用于当前位置。
      wco[idx] = gc_state.coord_system[idx] + gc_state.coord_offset[idx];
      if (idx == TOOL_LENGTH_OFFSET_AXIS)
      {
        wco[idx] += gc_state.tool_length_offset;
      }
      if (bit_isfalse(settings.status_report_mask, BITFLAG_RT_STATUS_POSITION_TYPE))
      {
        offset_position[idx] -= wco[idx];
      }
    }
  }

  // 报告机器位置
  // if (bit_istrue(settings.status_report_mask, BITFLAG_RT_STATUS_POSITION_TYPE))
  // {
  //   printPgmString(PSTR("|MPos:"));
  // }
  // else
  // {
  //   printPgmString(PSTR("|WPos:"));
  // }
  // report_util_axis_values(print_position);
  printPgmString(PSTR("|MPos:"));
  report_util_axis_values(print_position);
  printPgmString(PSTR("|WPos:"));
  report_util_axis_values(offset_position);

  printPgmString(PSTR("|T:"));
  print_uint8_base10(settings.tool);
  printPgmString(PSTR("|F:"));
  printFloat_RateValue(gc_state.feed_rate);
  printPgmString(PSTR("|ST:"));
  printFloat_RateValue(temp_obj.spindle_temp);
  printPgmString(PSTR("|S:"));
  printFloat(gc_state.spindle_speed, N_DECIMAL_RPMVALUE);
  printPgmString(PSTR("|M:"));
  switch (gc_state.modal.spindle)
  {
  case SPINDLE_ENABLE_CW:
    serial_write('3');
    break;
  case SPINDLE_ENABLE_CCW:
    serial_write('4');
    break;
  case SPINDLE_DISABLE:
    serial_write('5');
    break;
  }
  printPgmString(PSTR("|G:"));
  print_uint8_base10(gc_state.modal.coord_select + 54);

// 返回规划器和串行读取缓冲区状态。
#ifdef REPORT_FIELD_BUFFER_STATE
  if (bit_istrue(settings.status_report_mask, BITFLAG_RT_STATUS_BUFFER_STATE))
  {
    printPgmString(PSTR("|Bf:"));
    print_uint8_base10(plan_get_block_buffer_available());
    serial_write(',');
    print_uint8_base10(serial_get_rx_buffer_available());
  }
#endif

#ifdef REPORT_FIELD_LINE_NUMBERS
  // 报告当前行号
  plan_block_t *cur_block = plan_get_current_block();
  if (cur_block != NULL)
  {
    uint32_t ln = cur_block->line_number;
    if (ln > 0)
    {
      printPgmString(PSTR("|Ln:"));
      printInteger(ln);
    }
  }
#endif

// 报告实时进给速度
#ifdef REPORT_FIELD_CURRENT_FEED_SPEED
  printPgmString(PSTR("|FS:"));
  printFloat_RateValue(st_get_realtime_rate());
  serial_write(',');
  printFloat(sys.spindle_speed, N_DECIMAL_RPMVALUE);
#endif

#ifdef REPORT_FIELD_PIN_STATE
  uint8_t lim_pin_state = limits_get_state();
  uint8_t ctrl_pin_state = system_control_get_state();
  uint8_t prb_pin_state = probe_get_state();
  if (lim_pin_state | ctrl_pin_state | prb_pin_state)
  {
    printPgmString(PSTR("|Pn:"));
    if (prb_pin_state)
    {
      serial_write('P');
    }
    if (lim_pin_state)
    {
      if (bit_istrue(lim_pin_state, bit(X_AXIS)))
      {
        serial_write('X');
      }
      if (bit_istrue(lim_pin_state, bit(Y_AXIS)))
      {
        serial_write('Y');
      }
      if (bit_istrue(lim_pin_state, bit(Z_AXIS)))
      {
        serial_write('Z');
      }
#ifdef A_AXIS
      if (bit_istrue(lim_pin_state, bit(A_AXIS)))
      {
        serial_write('A');
      }
#endif
#ifdef B_AXIS
      if (bit_istrue(lim_pin_state, bit(B_AXIS)))
      {
        serial_write('B');
      }
#endif
#ifdef C_AXIS
      if (bit_istrue(lim_pin_state, bit(C_AXIS)))
      {
        serial_write('C');
      }
#endif
#ifdef D_AXIS
      if (bit_istrue(lim_pin_state, bit(D_AXIS)))
      {
        serial_write('D');
      }
#endif
    }
    if (ctrl_pin_state)
    {
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
      if (bit_istrue(ctrl_pin_state, CONTROL_PIN_INDEX_SAFETY_DOOR))
      {
        serial_write('D');
      }
#endif
      if (bit_istrue(ctrl_pin_state, CONTROL_PIN_INDEX_RESET))
      {
        serial_write('R');
      }
      if (bit_istrue(ctrl_pin_state, CONTROL_PIN_INDEX_FEED_HOLD))
      {
        serial_write('H');
      }
      if (bit_istrue(ctrl_pin_state, CONTROL_PIN_INDEX_CYCLE_START))
      {
        serial_write('S');
      }
    }
  }
#endif

#ifdef REPORT_FIELD_WORK_COORD_OFFSET
  if (sys.report_wco_counter > 0)
  {
    sys.report_wco_counter--;
  }
  else
  {
    if (sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR))
    {
      sys.report_wco_counter = (REPORT_WCO_REFRESH_BUSY_COUNT - 1); // 为缓慢刷新重置计数器
    }
    else
    {
      sys.report_wco_counter = (REPORT_WCO_REFRESH_IDLE_COUNT - 1);
    }
    if (sys.report_ovr_counter == 0)
    {
      sys.report_ovr_counter = 1;
    } // 在下一次报告时设置覆盖。
    // printPgmString(PSTR("|WCO:"));
    // report_util_axis_values(wco);
  }
#endif

#ifdef REPORT_FIELD_OVERRIDES
  if (sys.report_ovr_counter > 0)
  {
    sys.report_ovr_counter--;
  }
  else
  {
    if (sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR))
    {
      sys.report_ovr_counter = (REPORT_OVR_REFRESH_BUSY_COUNT - 1); // 为缓慢刷新重置计数器
    }
    else
    {
      sys.report_ovr_counter = (REPORT_OVR_REFRESH_IDLE_COUNT - 1);
    }
    // printPgmString(PSTR("|Ov:"));
    // print_uint8_base10(sys.f_override);
    // serial_write(',');
    // print_uint8_base10(sys.r_override);
    // serial_write(',');
    // print_uint8_base10(sys.spindle_speed_ovr);
  }
#endif

  serial_write('>');
  report_util_line_feed();
}

#ifdef DEBUG
void report_realtime_debug()
{
}
#endif
