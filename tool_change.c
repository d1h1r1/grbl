#include "grbl.h"

uint8_t tool_status = 0; // 0 松刀，1紧刀
char x_char[20], y_char[20], z_char[20], command[80];
void tool_home(uint8_t flag);
void set_tool_length();
void tool_length_zero();
void tool_control_init()
{
  // 换刀限位
  DDRL &= ~((1 << 6) | (1 << 7)); // 设置为输入引脚
  PORTL |= ((1 << 6) | (1 << 7)); // 启用内部上拉电阻。正常高操作。

  // 换刀检测
  DDRD &= ~(1 << 7); // 设置为输入引脚
  PORTD |= (1 << 7); // 启用内部上拉电阻。正常高操作。
}

void return_tool()
{
  printPgmString(PSTR("前刀号:"));
  printInteger(settings.tool);
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  if (settings.tool != 0)
  {
    // 移动到要还刀的xy位置
    float2string(settings.tool_x[settings.tool - 1], x_char, 3);
    float2string(settings.tool_y[settings.tool - 1], y_char, 3);
    sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
    gc_execute_line(command);
    // 下降到还刀位置
    float2string(settings.tool_z[settings.tool - 1], z_char, 3);
    sprintf(command, "G90G53G01Z%sF1000", z_char);
    gc_execute_line(command);
    // 松刀
    tool_home(1);
    // 抬刀
    gc_execute_line("G90G53G0Z-5");
  }
}

void get_tool(uint8_t tool_number)
{
  // 移动取刀位置
  float2string(settings.tool_x[tool_number - 1], x_char, 3);
  float2string(settings.tool_y[tool_number - 1], y_char, 3);
  sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
  gc_execute_line(command);
  // 下降到取刀位置
  float2string(settings.tool_z[tool_number - 1], z_char, 3);
  sprintf(command, "G90G53G01Z%sF1000", z_char);
  gc_execute_line(command);
  // 紧刀
  delay_ms(100);
  tool_home(0);
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
}

void change_tool(uint8_t tool_number)
{
  uint8_t beforeTool = settings.tool;
  if (tool_number == 0)
  {
    return_tool();
  }
  else
  {
    return_tool();
    get_tool(tool_number);
    set_tool_length();
    // if (beforeTool == 0)
    // {
    //   tool_length_zero();
    // }
    // else
    // {
    //   set_tool_length();
    // }
  }
  protocol_buffer_synchronize();
  // 将换完刀后刀号保存
  settings.tool = tool_number;
  write_global_settings(); // 将更新后的刀号写入eeprom
  printPgmString(PSTR("后刀号:"));
  printInteger(tool_number);
  printPgmString(PSTR("\r\n"));
}

//  1松 0紧
void tool_home(uint8_t flag)
{
  protocol_buffer_synchronize();
  uint8_t cycle_mask = 1 << B_AXIS;
  uint8_t idx = 4;
  if (sys.abort)
  {
    return;
  } // 如果已发出系统重置，则阻止。

  // 初始化用于回原点运动的计划数据结构。禁用主轴和冷却。
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data, 0, sizeof(plan_line_data_t));
  pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
  pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;

  // 初始化用于回原点计算的变量。
  uint8_t n_cycle = (2 * N_HOMING_LOCATE_CYCLE * 0 + 1);
  uint8_t step_pin[N_AXIS];
  float target[N_AXIS];
  float max_travel = 0.0;
  // 初始化步进引脚掩码
  step_pin[idx] = get_step_pin_mask(idx);
  if (bit_istrue(cycle_mask, bit(idx)))
  {
    // 基于 max_travel 设置目标。确保限位开关在搜索倍增器的影响下被激活。
    // 注意：settings.max_travel[] 存储为负值。
    max_travel = max(max_travel, (-3) * settings.max_travel[idx]);
  }

  float homing_rate = settings.homing_seek_rate;

  uint8_t limit_state, axislock, n_active_axis;

  system_convert_array_steps_to_mpos(target, sys_position);

  // 初始化并声明回原点例程所需的变量。
  axislock = 0;
  n_active_axis = 0;
  // 为活动轴设置目标位置并设置回原点速率的计算。
  if (bit_istrue(cycle_mask, bit(idx)))
  {
    n_active_axis++;
    sys_position[idx] = 0;
    // 根据循环掩码和回原点循环接近状态设置目标方向。
    // 注意：这样编译出来的代码比尝试过的任何其他实现都要小。
    if (flag)
    {
      target[idx] = max_travel;
    }
    else
    {
      target[idx] = -max_travel;
    }
    // 将轴锁应用于本循环中活动的步进端口引脚。
    axislock |= step_pin[idx];
  }
  homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] 调整以便每个轴都以回原点速率移动。
  sys.homing_axis_lock = axislock;

  // 执行回原点循环。计划器缓冲区应为空，以便启动回原点循环。
  pl_data->feed_rate = 100;          // 设置当前回原点速率。
  plan_buffer_line(target, pl_data); // 绕过 mc_line()。直接计划回原点运动。

  sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; // 设置为执行回原点运动并清除现有标志。
  st_prep_buffer();                                   // 准备并填充段缓冲区，来源于新计划的块。
  st_wake_up();                                       // 启动运动
  do
  {
    // 检查限位状态。当它们发生变化时锁定循环轴。
    if (flag)
    {
      limit_state = ~PINL & (1 << 6);
    }
    else
    {
      limit_state = ~PINL & (1 << 7);
    }
    if (axislock & step_pin[idx])
    {
      if (limit_state)
      {
        axislock &= ~(step_pin[idx]);
      }
    }
    sys.homing_axis_lock = axislock;

    st_prep_buffer(); // 检查并准备段缓冲区。注意：此操作应不超过 200 微秒。
    // 退出例程：在此循环中没有时间运行 protocol_execute_realtime()。
    if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP))
    {
      uint8_t rt_exec = sys_rt_exec_state;
      // 回原点失败条件：在循环中发出重置。
      if (rt_exec & EXEC_RESET)
      {
        system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET);
      }
      // 回原点失败条件：安全门已打开。
      if (rt_exec & EXEC_SAFETY_DOOR)
      {
        system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR);
      }
      if (sys_rt_exec_alarm)
      {
        mc_reset(); // 停止电机（如果正在运行）。
        protocol_execute_realtime();
        return;
      }
      else
      {
        // 拉离运动完成。禁用执行的 CYCLE_STOP。
        system_clear_exec_state_flag(EXEC_CYCLE_STOP);
        break;
      }
    }
  } while (STEP_MASK & axislock);

  st_reset();                               // 立即强制停止步进电机并重置步进段缓冲区。
  delay_ms(settings.homing_debounce_delay); // 延迟以允许瞬态动态衰减。
  max_travel = settings.homing_pulloff * 3.0;
  homing_rate = settings.homing_feed_rate;

  sys_position[idx] = 0;
  sys.step_control = STEP_CONTROL_NORMAL_OP; // 将步进控制返回到正常操作。
  protocol_execute_realtime();               // 检查重置并设置系统中止。
  if (sys.abort)
  {
    return;
  } // 未完成。由 mc_alarm 设置的警报状态。

  // 归零循环完成！设置系统以正常运行。
  // -------------------------------------------------------------------------------------

  // 同步 G-code 解析器和规划器位置到归零位置。
  gc_sync_position();
  plan_sync_position();
}

// 校准刀具长度
void tool_length_zero()
{
  printPgmString(PSTR("开始对刀"));
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G01Z-5F1000");
  // 移动到对刀的xy位置
  float2string(settings.tool_x[TOOL_NUM - 1], x_char, 3);
  float2string(settings.tool_y[TOOL_NUM - 1], y_char, 3);
  sprintf(command, "G90G53G01X%sY%sF1000", x_char, y_char);
  gc_execute_line(command);
  // 下降到对刀z位置
  float2string(settings.tool_z[TOOL_NUM - 1], z_char, 3);
  sprintf(command, "G90G53G01Z%sF1000", z_char);
  gc_execute_line(command);
  gc_execute_line("G21G91G38.2Z-100F200");
  gc_execute_line("G0Z1");
  gc_execute_line("G38.2Z-2F30");

  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position, sys_position);
  settings.tool_zpos = print_position[2];
  settings.tool_length = 0;
  gc_state.tool_length_offset = 0;
  write_global_settings(); // 将更新后的刀长写入eeprom
  // report_probe_parameters();
  gc_execute_line("G90G53G01Z-5F1000");
}

// 设置刀补
void set_tool_length()
{
  printPgmString(PSTR("开始对刀"));
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G01Z-5F1000");
  // 移动到对刀的xy位置
  float2string(settings.tool_x[TOOL_NUM - 1], x_char, 3);
  float2string(settings.tool_y[TOOL_NUM - 1], y_char, 3);
  sprintf(command, "G90G53G01X%sY%sF1000", x_char, y_char);
  gc_execute_line(command);
  // 下降到对刀z位置
  float2string(settings.tool_z[TOOL_NUM - 1], z_char, 3);
  sprintf(command, "G90G53G01Z%sF1000", z_char);
  gc_execute_line(command);
  gc_execute_line("G21G91G38.2Z-100F200");
  gc_execute_line("G0Z1");
  gc_execute_line("G38.2Z-2F30");
  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position, sys_position);
  gc_state.tool_length_offset = print_position[2] - settings.tool_zpos + settings.tool_length;
  settings.tool_length = gc_state.tool_length_offset;
  settings.tool_zpos = print_position[2];
  write_global_settings(); // 将更新后的刀长写入eeprom
  // report_probe_parameters();
  // 抬刀
  gc_execute_line("G90G53G01Z-5F1000");
}