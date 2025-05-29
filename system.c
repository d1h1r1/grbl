/*
  system.c - 处理系统级命令和实时过程
  Grbl 的一部分

  版权所有 (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

void system_init()
{
  CONTROL_DDR &= ~(CONTROL_MASK); // 配置为输入引脚
#ifdef DISABLE_CONTROL_PIN_PULL_UP
  CONTROL_PORT &= ~(CONTROL_MASK); // 常规低操作。需要外部下拉。
#else
  CONTROL_PORT |= CONTROL_MASK; // 启用内部上拉电阻。常规高操作。
#endif
  CONTROL_PCMSK |= CONTROL_MASK; // 启用引脚变化中断的特定引脚
  PCICR |= (1 << CONTROL_INT);   // 启用引脚变化中断
}

// 返回控制引脚状态作为 uint8 位域。每个位表示输入引脚状态，其中
// 被触发为 1，未被触发为 0。应用反转掩码。位域组织由
// 头文件中的 CONTROL_PIN_INDEX 定义。
uint8_t system_control_get_state()
{
  uint8_t control_state = 0;
  uint8_t pin = (CONTROL_PIN & CONTROL_MASK);
#ifdef INVERT_CONTROL_PIN_MASK
  pin ^= INVERT_CONTROL_PIN_MASK;
#endif
  if (pin)
  {
    if (bit_isfalse(pin, (1 << CONTROL_SAFETY_DOOR_BIT)))
    {
      control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR;
    }
    if (bit_isfalse(pin, (1 << CONTROL_RESET_BIT)))
    {
      control_state |= CONTROL_PIN_INDEX_RESET;
    }
    if (bit_isfalse(pin, (1 << CONTROL_FEED_HOLD_BIT)))
    {
      control_state |= CONTROL_PIN_INDEX_FEED_HOLD;
    }
    if (bit_isfalse(pin, (1 << CONTROL_CYCLE_START_BIT)))
    {
      control_state |= CONTROL_PIN_INDEX_CYCLE_START;
    }
  }
  return (control_state);
}

// 引脚变化中断，用于引脚输出命令，即循环开始、进给保持和重置。
// 仅设置实时命令执行变量，以便在主程序准备好时执行这些命令。
// 这与从输入串行数据流直接提取的字符型实时命令的工作方式相同。
ISR(CONTROL_INT_vect)
{
  uint8_t pin = system_control_get_state();
  if (pin)
  {
    if (bit_istrue(pin, CONTROL_PIN_INDEX_RESET))
    {
      mc_reset();
    }
    else if (bit_istrue(pin, CONTROL_PIN_INDEX_CYCLE_START))
    {
      bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
    }
    else if (bit_istrue(pin, CONTROL_PIN_INDEX_FEED_HOLD))
    {
      bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
    }
    else if (bit_istrue(pin, CONTROL_PIN_INDEX_SAFETY_DOOR))
    {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
    }
  }
}

// 返回安全门是否开启（T）或关闭（F），基于引脚状态。
uint8_t system_check_safety_door_ajar()
{
  return (system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
}

// 执行用户启动脚本（如果已存储）。
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n = 0; n < N_STARTUP_LINE; n++)
  {
    if (!(settings_read_startup_line(n, line)))
    {
      line[0] = 0;
      report_execute_startup_message(line, STATUS_SETTING_READ_FAIL);
    }
    else
    {
      if (line[0] != 0)
      {
        uint8_t status_code = gc_execute_line(line);
        report_execute_startup_message(line, status_code);
      }
    }
  }
}

// 指导并执行来自 protocol_process 的一行格式化输入。虽然主要是
// 输入流 g-code 块，但这也执行 Grbl 内部命令，例如
// 设置、启动归位循环和切换开关状态。这与
// 实时命令模块不同，因为它取决于 Grbl 准备执行下一行的时机，
// 因此对于像块删除这样的开关，仅影响后续处理的行，而不一定是实时
// 在循环过程中，因为已经在缓冲区中存储了运动。不过，这种“延迟”
// 不应该成为问题，因为这些命令在循环过程中通常不使用。

uint8_t system_execute_line(char *line)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // 辅助变量
  float parameter, value;
  switch (line[char_counter])
  {
  case 0:
    report_grbl_help();
    break; // 显示 Grbl 帮助
  // case 'D':
  //   tool_home(1);
  //   break; // 松刀
  // case 'U':
  //   tool_home(0);
  //   break; // 紧刀
  case 'P':
    set_probe(1);
    break;
  case 'V':
    set_probe(0);
    break;
  case 'A':
    set_laser(0);
    break;
  case 'B':
    set_laser(1);
    break;
  case 'E':
    tool_length_zero();
    break;
  case 'F':
    getToolStatus();
    break;
  case 'T':
    report_tool();
    break;
  case 'J': // 手动移动
    // 仅在 IDLE 或 JOG 状态下执行。
    if (sys.state != STATE_IDLE && sys.state != STATE_JOG)
    {
      return (STATUS_IDLE_ERROR);
    }
    if (line[2] != '=')
    {
      return (STATUS_INVALID_STATEMENT);
    }
    return (gc_execute_line(line)); // 注意：$J= 在 g-code 解析器中被忽略，并用于检测手动移动。
    break;
  case '$':
  case 'G':
  case 'C':
  case 'X':
    if (line[2] != 0)
    {
      return (STATUS_INVALID_STATEMENT);
    }
    switch (line[1])
    {
    case '$': // 打印 Grbl 设置
      if (sys.state & (STATE_CYCLE | STATE_HOLD))
      {
        return (STATUS_IDLE_ERROR);
      } // 在循环期间阻止。打印时间过长。
      else
      {
        report_grbl_settings();
      }
      break;
    case 'G': // 打印 gcode 解析器状态
      // TODO: 将此移动到实时命令，以便 GUI 在暂停状态时请求此数据。
      report_gcode_modes();
      break;
    case 'C': // 设置检查 g-code 模式 [IDLE/CHECK]
      // 切换关闭时执行重置。检查 g-code 模式应仅在 Grbl 空闲且准备好时工作，无论警报锁定状态如何。这主要是为了保持简单和一致。
      if (sys.state == STATE_CHECK_MODE)
      {
        mc_reset();
        report_feedback_message(MESSAGE_DISABLED);
      }
      else
      {
        if (sys.state)
        {
          return (STATUS_IDLE_ERROR);
        } // 需要无警报模式。
        sys.state = STATE_CHECK_MODE;
        report_feedback_message(MESSAGE_ENABLED);
      }
      break;
    case 'X': // 禁用警报锁定 [ALARM]
      if (sys.state == STATE_ALARM)
      {
        // 如果安全门未关闭，则阻止。
        if (system_check_safety_door_ajar())
        {
          return (STATUS_CHECK_DOOR);
        }
        report_feedback_message(MESSAGE_ALARM_UNLOCK);
        sys.state = STATE_IDLE;
        // 不运行启动脚本。防止启动中的存储移动造成事故。
      } // 否则无效。
      break;
    }
    break;
  default:
    // 阻止任何需要状态为 IDLE/ALARM 的系统命令。（即 EEPROM，归位）
    if (!(sys.state == STATE_IDLE || sys.state == STATE_ALARM))
    {
      return (STATUS_IDLE_ERROR);
    }
    switch (line[1])
    {
    case '#': // 打印 Grbl NGC 参数
      if (line[2] != 0)
      {
        return (STATUS_INVALID_STATEMENT);
      }
      else
      {
        report_ngc_parameters();
      }
      break;
    case 'H': // 执行归位循环 [IDLE/ALARM]
      if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE))
      {
        return (STATUS_SETTING_DISABLED);
      }
      if (system_check_safety_door_ajar())
      {
        return (STATUS_CHECK_DOOR);
      } // 如果安全门未关闭，则阻止。
      sys.state = STATE_HOMING; // 设置系统状态变量
      if (line[2] == 0)
      {
        mc_homing_cycle(HOMING_CYCLE_ALL);
#ifdef HOMING_SINGLE_AXIS_COMMANDS
      }
      else if (line[3] == 0)
      {
        switch (line[2])
        {
        case 'X':
          mc_homing_cycle(HOMING_CYCLE_X);
          break;
        case 'Y':
          mc_homing_cycle(HOMING_CYCLE_Y);
          break;
        case 'Z':
          mc_homing_cycle(HOMING_CYCLE_Z);
          break;
        default:
          return (STATUS_INVALID_STATEMENT);
        }
#endif
      }
      else
      {
        return (STATUS_INVALID_STATEMENT);
      }
      if (!sys.abort)
      {                         // 在成功归位后执行启动脚本。
        sys.state = STATE_IDLE; // 完成时设置为 IDLE。
        st_go_idle();           // 在返回之前将步进电机设置为设置的空闲状态。
        if (line[2] == 0)
        {
          system_execute_startup(line);
        }
      }
      break;
    case 'S': // 将 Grbl 置于睡眠状态 [IDLE/ALARM]
      if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0))
      {
        return (STATUS_INVALID_STATEMENT);
      }
      system_set_exec_state_flag(EXEC_SLEEP); // 立即设置为执行睡眠模式
      break;
    case 'I': // 打印或存储构建信息 [IDLE/ALARM]
      if (line[++char_counter] == 0)
      {
        settings_read_build_info(line);
        report_build_info(line);
#ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
      }
      else
      { // 存储启动行 [IDLE/ALARM]
        if (line[char_counter++] != '=')
        {
          return (STATUS_INVALID_STATEMENT);
        }
        helper_var = char_counter; // 将辅助变量设置为用户信息行的开始计数器。
        do
        {
          line[char_counter - helper_var] = line[char_counter];
        } while (line[char_counter++] != 0);
        settings_store_build_info(line);
#endif
      }
      break;
    case 'R': // 恢复默认设置 [IDLE/ALARM]
      if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0))
      {
        return (STATUS_INVALID_STATEMENT);
      }
      switch (line[5])
      {
#ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
      case '$':
        settings_restore(SETTINGS_RESTORE_DEFAULTS);
        break;
#endif
#ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
      case '#':
        settings_restore(SETTINGS_RESTORE_PARAMETERS);
        break;
#endif
#ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
      case '*':
        settings_restore(SETTINGS_RESTORE_ALL);
        break;
#endif
      default:
        return (STATUS_INVALID_STATEMENT);
      }
      report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
      mc_reset(); // 强制重置以确保设置正确初始化。
      break;
    case 'N': // 启动行 [IDLE/ALARM]
      if (line[++char_counter] == 0)
      { // 打印启动行
        for (helper_var = 0; helper_var < N_STARTUP_LINE; helper_var++)
        {
          if (!(settings_read_startup_line(helper_var, line)))
          {
            report_status_message(STATUS_SETTING_READ_FAIL);
          }
          else
          {
            report_startup_line(helper_var, line);
          }
        }
        break;
      }
      else
      { // 存储启动行 [仅 IDLE] 防止在警报期间移动。
        if (sys.state != STATE_IDLE)
        {
          return (STATUS_IDLE_ERROR);
        } // 仅在空闲时存储。
        helper_var = true; // 将辅助变量设置为标记存储方法。
        // 没有 break。继续进入 default: 以读取剩余的命令字符。
      }
    default: // 存储设置方法 [IDLE/ALARM]
      if (!read_float(line, &char_counter, &parameter))
      {
        return (STATUS_BAD_NUMBER_FORMAT);
      }
      if (line[char_counter++] != '=')
      {
        return (STATUS_INVALID_STATEMENT);
      }
      if (helper_var)
      { // 存储启动行
        // 通过移动所有字符准备将 gcode 块发送到 gcode 解析器
        helper_var = char_counter; // 将辅助变量设置为 gcode 块的起始计数器
        do
        {
          line[char_counter - helper_var] = line[char_counter];
        } while (line[char_counter++] != 0);
        if (char_counter > EEPROM_LINE_SIZE)
        {
          return (STATUS_LINE_LENGTH_EXCEEDED);
        }
        // 执行 gcode 块以确保块有效。
        helper_var = gc_execute_line(line); // 将辅助变量设置为返回的状态代码。
        if (helper_var)
        {
          return (helper_var);
        }
        else
        {
          helper_var = trunc(parameter); // 将辅助变量设置为参数的整数值
          settings_store_startup_line(helper_var, line);
        }
      }
      else
      { // 存储全局设置。
        if (!read_float(line, &char_counter, &value))
        {
          return (STATUS_BAD_NUMBER_FORMAT);
        }
        if ((line[char_counter] != 0) || (parameter > 255))
        {
          return (STATUS_INVALID_STATEMENT);
        }
        return (settings_store_global_setting((uint8_t)parameter, value));
      }
    }
  }
  return (STATUS_OK); // 如果 '$' 命令能到这里，则一切正常。
}

void system_flag_wco_change()
{
#ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
  protocol_buffer_synchronize(); // 在 WCO 变化期间强制同步缓冲区
#endif
  sys.report_wco_counter = 0; // 重置 WCO 计数器
}

// 返回轴 'idx' 的机器位置。必须传入一个 'step' 数组。
// 注意：如果电机步进和机器位置不在同一坐标系中，此函数
//   用作计算变换的中心位置。
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
#ifdef COREXY
  if (idx == X_AXIS)
  {
    pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx]; // 将 CoreXY 步进转换为 X 轴位置
  }
  else if (idx == Y_AXIS)
  {
    pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx]; // 将 CoreXY 步进转换为 Y 轴位置
  }
  else
  {
    pos = steps[idx] / settings.steps_per_mm[idx]; // 对于其他轴，直接计算位置
  }
#else
  pos = steps[idx] / settings.steps_per_mm[idx]; // 对于非 CoreXY 系统，直接计算位置
#endif
  return (pos); // 返回计算得到的位置
}

void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx = 0; idx < N_AXIS; idx++)
  {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx); // 将每个轴的步进转换为机器位置
  }
  return;
}

// 仅用于 CoreXY 计算。根据 CoreXY 电机步进返回 X 或 Y 轴的“步进”。
#ifdef COREXY
int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
{
  return ((steps[A_MOTOR] + steps[B_MOTOR]) / 2); // 计算 X 轴步进
}
int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
{
  return ((steps[A_MOTOR] - steps[B_MOTOR]) / 2); // 计算 Y 轴步进
}
#endif

// 检查并报告目标数组是否超过机器移动限制。
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx = 0; idx < N_AXIS; idx++)
  {
    // 允许通过将最大移动限制设置为零来禁用每个轴的软限制
    if (settings.max_travel[idx])
    {
#ifdef HOMING_FORCE_SET_ORIGIN
      // 当强制归位设置原点启用时，软限制检查需要考虑方向性。
      // 注意：最大移动限制存储为负值
      if (bit_istrue(settings.homing_dir_mask, bit(idx)))
      {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx])
        {
          return (true);
        }
      }
      else
      {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx])
        {
          return (true);
        }
      }
#else
      // 注意：最大移动限制存储为负值
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx])
      {
        return (true);
      }
#endif
    }
  }
  return (false); // 没有超过限制
}

// 用于设置和清除 Grbl 实时执行标志的特殊处理程序。
void system_set_exec_state_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                       // 禁用中断
  sys_rt_exec_state |= (mask); // 设置执行状态标志
  SREG = sreg;                 // 恢复中断状态
}

void system_clear_exec_state_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                        // 禁用中断
  sys_rt_exec_state &= ~(mask); // 清除执行状态标志
  SREG = sreg;                  // 恢复中断状态
}

void system_set_exec_alarm(uint8_t code)
{
  uint8_t sreg = SREG;
  cli();                    // 禁用中断
  sys_rt_exec_alarm = code; // 设置执行警报
  SREG = sreg;              // 恢复中断状态
}

void system_clear_exec_alarm()
{
  uint8_t sreg = SREG;
  cli();                 // 禁用中断
  sys_rt_exec_alarm = 0; // 清除执行警报
  SREG = sreg;           // 恢复中断状态
}

void system_set_exec_motion_override_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                                 // 禁用中断
  sys_rt_exec_motion_override |= (mask); // 设置运动覆盖标志
  SREG = sreg;                           // 恢复中断状态
}

void system_set_exec_accessory_override_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                                    // 禁用中断
  sys_rt_exec_accessory_override |= (mask); // 设置附属设备覆盖标志
  SREG = sreg;                              // 恢复中断状态
}

void system_clear_exec_motion_overrides()
{
  uint8_t sreg = SREG;
  cli();                           // 禁用中断
  sys_rt_exec_motion_override = 0; // 清除运动覆盖
  SREG = sreg;                     // 恢复中断状态
}

void system_clear_exec_accessory_overrides()
{
  uint8_t sreg = SREG;
  cli();                              // 禁用中断
  sys_rt_exec_accessory_override = 0; // 清除附属设备覆盖
  SREG = sreg;                        // 恢复中断状态
}
