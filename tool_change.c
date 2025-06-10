#include "grbl.h"

uint8_t tool_status = 0; // 0 松刀，1紧刀
char x_char[20], y_char[20], z_char[20], command[80];
void set_tool_length();
void tool_length_zero();

void tool_control_init()
{
  // 换刀检测
  DDRF &= ~(1 << 6); // 设置为输入引脚
  PORTF |= (1 << 6); // 启用内部上拉电阻。正常高操作。
  // PORTF &= ~(1 << 6); // 正常低操作。需要外部下拉。
}

void return_tool()
{
  printPgmString(PSTR("前刀号:"));
  printInteger(settings.tool);
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  gc_execute_line("M4S2300");
  protocol_buffer_synchronize();
  if (settings.tool != 0)
  {
    // 移动到要还刀的xy位置
    float2string(settings.tool_x[settings.tool - 1], x_char, 3);
    float2string(settings.tool_y[settings.tool - 1], y_char, 3);
    sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
    gc_execute_line(command);
    // 下降到还刀位置
    float2string(settings.tool_z[settings.tool - 1], z_char, 3);
    sprintf(command, "G90G53G01Z%sF1200", z_char);
    gc_execute_line(command);
    // 松刀
    // 抬刀
    gc_execute_line("G90G53G0Z-5");
    protocol_buffer_synchronize();
    gc_execute_line("M5");
  }
}

void getToolStatus(){
  printPgmString(PSTR("[换刀状态: "));
  // uint8_t status = PINF & (1 << 6);
  print_uint8_base10((PINF & (1 << 6)) ? 1 : 0);
  printPgmString(PSTR("]//"));
  printPgmString(PSTR("\r\n"));
  return 0;
}

void get_tool(uint8_t tool_number)
{
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  protocol_buffer_synchronize();
  gc_execute_line("M3S1800");
  // 移动取刀位置
  float2string(settings.tool_x[tool_number - 1], x_char, 3);
  float2string(settings.tool_y[tool_number - 1], y_char, 3);
  sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
  gc_execute_line(command);
  // 下降到取刀位置
  float2string(settings.tool_z[tool_number - 1], z_char, 3);
  sprintf(command, "G90G53G01Z%sF1200", z_char);
  gc_execute_line(command);
  protocol_buffer_synchronize();
  delay_ms(500);
  // 紧刀
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  protocol_buffer_synchronize();
  gc_execute_line("M5");
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