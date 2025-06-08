/*
  settings.c - EEPROM 配置处理
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证进行再发行和/或修改，
  该许可证的版本为 3，或（根据您的选择）任何后续版本。

  Grbl 的分发是出于它会有用的希望，
  但没有任何担保；甚至没有适销性或适合特定目的的隐含担保。有关更多详细信息，请参见
  GNU 通用公共许可证。

  您应该已收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

settings_t settings;

// 将启动行存储到 EEPROM 的方法
void settings_store_startup_line(uint8_t n, char *line)
{
#ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
  protocol_buffer_synchronize(); // 启动行可能包含运动并正在执行。
#endif
  uint32_t addr = n * (LINE_BUFFER_SIZE + 1) + EEPROM_ADDR_STARTUP_BLOCK;
  memcpy_to_eeprom_with_checksum(addr, (char *)line, LINE_BUFFER_SIZE);
}

// 将构建信息存储到 EEPROM 的方法
// 注意：此函数只能在 IDLE 状态下调用。
void settings_store_build_info(char *line)
{
  // 仅当状态为 IDLE 时才能存储构建信息。
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_BUILD_INFO, (char *)line, LINE_BUFFER_SIZE);
}

// 将坐标数据参数存储到 EEPROM 的方法
void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{
#ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
  protocol_buffer_synchronize();
#endif
  uint32_t addr = coord_select * (sizeof(float) * N_AXIS + 1) + EEPROM_ADDR_PARAMETERS;
  memcpy_to_eeprom_with_checksum(addr, (char *)coord_data, sizeof(float) * N_AXIS);
}

// 将 Grbl 全局设置结构和版本号存储到 EEPROM 的方法
// 注意：此函数只能在 IDLE 状态下调用。
void write_global_settings()
{
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char *)&settings, sizeof(settings_t));
}

// 将 EEPROM 保存的 Grbl 全局设置恢复为默认值的方法。
void settings_restore(uint8_t restore_flag)
{
  if (restore_flag & SETTINGS_RESTORE_DEFAULTS)
  {
    settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
    settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
    settings.step_invert_mask = DEFAULT_STEPPING_INVERT_MASK;
    settings.dir_invert_mask = DEFAULT_DIRECTION_INVERT_MASK;
    settings.status_report_mask = DEFAULT_STATUS_REPORT_MASK;
    settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
    settings.arc_tolerance = DEFAULT_ARC_TOLERANCE;

    settings.rpm_max = DEFAULT_SPINDLE_RPM_MAX;
    settings.rpm_min = DEFAULT_SPINDLE_RPM_MIN;

    settings.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
    settings.homing_feed_rate = DEFAULT_HOMING_FEED_RATE;
    settings.homing_seek_rate = DEFAULT_HOMING_SEEK_RATE;
    settings.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
    settings.homing_pulloff = DEFAULT_HOMING_PULLOFF;

    settings.flags = 0;
    if (DEFAULT_REPORT_INCHES)
    {
      settings.flags |= BITFLAG_REPORT_INCHES;
    }
    if (DEFAULT_LASER_MODE)
    {
      settings.flags |= BITFLAG_LASER_MODE;
    }
    if (DEFAULT_INVERT_ST_ENABLE)
    {
      settings.flags |= BITFLAG_INVERT_ST_ENABLE;
    }
    if (DEFAULT_HARD_LIMIT_ENABLE)
    {
      settings.flags |= BITFLAG_HARD_LIMIT_ENABLE;
    }
    if (DEFAULT_HOMING_ENABLE)
    {
      settings.flags |= BITFLAG_HOMING_ENABLE;
    }
    if (DEFAULT_SOFT_LIMIT_ENABLE)
    {
      settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
    }
    if (DEFAULT_INVERT_LIMIT_PINS)
    {
      settings.flags |= BITFLAG_INVERT_LIMIT_PINS;
    }
    if (DEFAULT_INVERT_PROBE_PIN)
    {
      settings.flags |= BITFLAG_INVERT_PROBE_PIN;
    }

    settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
    settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
    settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
    settings.max_rate[X_AXIS] = DEFAULT_X_MAX_RATE;
    settings.max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE;
    settings.max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE;
    settings.acceleration[X_AXIS] = DEFAULT_X_ACCELERATION;
    settings.acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION;
    settings.acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION;
    settings.max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL);
    settings.max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL);
    settings.max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL);
    settings.tool = 1;
    settings.tool_length = 0;
    settings.tool_zpos = 0;
    for (size_t i = 0; i < TOOL_NUM; i++)
    {
      settings.tool_x[i] = -1;
      settings.tool_y[i] = i * 10;
      settings.tool_z[i] = i * 10;
    }
    memset(settings.tool_data, 0, sizeof(settings.tool_data));
#ifdef A_AXIS
    settings.steps_per_mm[A_AXIS] = DEFAULT_A_STEPS_PER_MM;
    settings.max_rate[A_AXIS] = DEFAULT_A_MAX_RATE;
    settings.acceleration[A_AXIS] = DEFAULT_A_ACCELERATION;
    settings.max_travel[A_AXIS] = (-DEFAULT_A_MAX_TRAVEL);
#endif
#ifdef B_AXIS
    settings.steps_per_mm[B_AXIS] = DEFAULT_B_STEPS_PER_MM;
    settings.max_rate[B_AXIS] = DEFAULT_B_MAX_RATE;
    settings.acceleration[B_AXIS] = DEFAULT_B_ACCELERATION;
    settings.max_travel[B_AXIS] = (-DEFAULT_B_MAX_TRAVEL);
#endif
#ifdef C_AXIS
    settings.steps_per_mm[C_AXIS] = DEFAULT_C_STEPS_PER_MM;
    settings.acceleration[C_AXIS] = DEFAULT_C_ACCELERATION;
    settings.max_rate[C_AXIS] = DEFAULT_C_MAX_RATE;
    settings.max_travel[C_AXIS] = (-DEFAULT_C_MAX_TRAVEL);
#endif
#ifdef D_AXIS
    settings.steps_per_mm[D_AXIS] = DEFAULT_D_STEPS_PER_MM;
    settings.acceleration[D_AXIS] = DEFAULT_D_ACCELERATION;
    settings.max_rate[D_AXIS] = DEFAULT_D_MAX_RATE;
    settings.max_travel[D_AXIS] = (-DEFAULT_D_MAX_TRAVEL);
#endif
    write_global_settings();
  }

  if (restore_flag & SETTINGS_RESTORE_PARAMETERS)
  {
    uint8_t idx;
    float coord_data[N_AXIS];
    memset(&coord_data, 0, sizeof(coord_data));
    for (idx = 0; idx <= SETTING_INDEX_NCOORD; idx++)
    {
      settings_write_coord_data(idx, coord_data);
    }
  }

  if (restore_flag & SETTINGS_RESTORE_STARTUP_LINES)
  {
#if N_STARTUP_LINE > 0
    eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK, 0);
    eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK + 1, 0); // 校验和
#endif
#if N_STARTUP_LINE > 1
    eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK + (LINE_BUFFER_SIZE + 1), 0);
    eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK + (LINE_BUFFER_SIZE + 2), 0); // 校验和
#endif
  }

  if (restore_flag & SETTINGS_RESTORE_BUILD_INFO)
  {
    eeprom_put_char(EEPROM_ADDR_BUILD_INFO, 0);
    eeprom_put_char(EEPROM_ADDR_BUILD_INFO + 1, 0); // 校验和
  }
}

// 从 EEPROM 读取启动行。更新指向的行字符串数据。
uint8_t settings_read_startup_line(uint8_t n, char *line)
{
  uint32_t addr = n * (LINE_BUFFER_SIZE + 1) + EEPROM_ADDR_STARTUP_BLOCK;
  if (!(memcpy_from_eeprom_with_checksum((char *)line, addr, LINE_BUFFER_SIZE)))
  {
    // 用默认值重置行
    line[0] = 0; // 空行
    settings_store_startup_line(n, line);
    return (false);
  }
  return (true);
}

// 从 EEPROM 读取构建信息。更新指向的行字符串数据。
uint8_t settings_read_build_info(char *line)
{
  if (!(memcpy_from_eeprom_with_checksum((char *)line, EEPROM_ADDR_BUILD_INFO, LINE_BUFFER_SIZE)))
  {
    // 用默认值重置行
    line[0] = 0; // 空行
    settings_store_build_info(line);
    return (false);
  }
  return (true);
}

// 从 EEPROM 读取所选坐标数据。更新指向的 coord_data 值。
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
  uint32_t addr = coord_select * (sizeof(float) * N_AXIS + 1) + EEPROM_ADDR_PARAMETERS;
  if (!(memcpy_from_eeprom_with_checksum((char *)coord_data, addr, sizeof(float) * N_AXIS)))
  {
    // 用默认零向量重置
    clear_vector_float(coord_data);
    settings_write_coord_data(coord_select, coord_data);
    return (false);
  }
  return (true);
}

// 从 EEPROM 读取 Grbl 全局设置结构。
uint8_t read_global_settings()
{
  // 检查 EEPROM 的版本字节
  uint8_t version = eeprom_get_char(0);
  if (version == SETTINGS_VERSION)
  {
    // 读取设置记录并检查校验和
    if (!(memcpy_from_eeprom_with_checksum((char *)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t))))
    {
      return (false);
    }
  }
  else
  {
    return (false);
  }
  return (true);
}

// 从命令行设置设置的辅助方法
uint8_t settings_store_global_setting(uint8_t parameter, float value)
{
  // if (value < 0.0)
  // {
  //   return (STATUS_NEGATIVE_VALUE);
  // }
  if (parameter < TOOL_SETTINGS_START_VAL && parameter >= 200)
  {
    switch (parameter)
    {
    case 200:
      // 储存刀号
      settings.tool = value;
      break;
    case 201:
      // 储存刀长
      settings.tool_length = value;
      gc_state.tool_length_offset = settings.tool_length;
      break;
    case 202:
      // 储存刀z位置
      settings.tool_zpos = value;
      break;
    }
  }
  else if (parameter >= TOOL_SETTINGS_START_VAL)
  {
    // 储存刀坐标
    parameter -= TOOL_SETTINGS_START_VAL; // 1 2 3 4 5 6
    uint8_t index = parameter / 3;
    uint8_t axis = parameter % 3;
    switch (axis)
    {
    case 0:
      settings.tool_x[index] = value;
      break;
    case 1:
      settings.tool_y[index] = value;
      break;
    case 2:
      settings.tool_z[index] = value;
      break;
    }
  }
  else if (parameter >= AXIS_SETTINGS_START_VAL)
  {
    // 存储轴配置。轴编号序列由 AXIS_SETTING 定义。
    // 注意：确保设置索引与 report.c 设置输出对应。
    parameter -= AXIS_SETTINGS_START_VAL;
    uint8_t set_idx = 0;
    while (set_idx < AXIS_N_SETTINGS)
    {
      if (parameter < N_AXIS)
      {
        // 找到有效的轴设置。
        switch (set_idx)
        {
        case 0:
#ifdef MAX_STEP_RATE_HZ
          if (value * settings.max_rate[parameter] > (MAX_STEP_RATE_HZ * 60.0))
          {
            return (STATUS_MAX_STEP_RATE_EXCEEDED);
          }
#endif
          settings.steps_per_mm[parameter] = value;
          break;
        case 1:
#ifdef MAX_STEP_RATE_HZ
          if (value * settings.steps_per_mm[parameter] > (MAX_STEP_RATE_HZ * 60.0))
          {
            return (STATUS_MAX_STEP_RATE_EXCEEDED);
          }
#endif
          settings.max_rate[parameter] = value;
          break;
        case 2:
          settings.acceleration[parameter] = value * 60 * 60;
          break; // 转换为 mm/min² 用于 Grbl 内部使用。
        case 3:
          settings.max_travel[parameter] = -value;
          break; // 以负值存储以供 Grbl 内部使用。
        }
        break; // 设置完成后退出循环，继续 EEPROM 写入调用。
      }
      else
      {
        set_idx++;
        // 如果轴索引大于 N_AXIS 或设置索引大于轴设置数，则返回错误。
        if ((parameter < AXIS_SETTINGS_INCREMENT) || (set_idx == AXIS_N_SETTINGS))
        {
          return (STATUS_INVALID_STATEMENT);
        }
        parameter -= AXIS_SETTINGS_INCREMENT;
      }
    }
  }
  else
  {
    // 存储非轴 Grbl 设置
    uint8_t int_value = trunc(value);
    switch (parameter)
    {
    case 0:
      if (int_value < 3)
      {
        return (STATUS_SETTING_STEP_PULSE_MIN);
      }
      settings.pulse_microseconds = int_value;
      break;
    case 1:
      settings.stepper_idle_lock_time = int_value;
      break;
    case 2:
      settings.step_invert_mask = int_value;
      st_generate_step_dir_invert_masks(); // 重新生成步进和方向端口反转掩码。
      break;
    case 3:
      settings.dir_invert_mask = int_value;
      st_generate_step_dir_invert_masks(); // 重新生成步进和方向端口反转掩码。
      break;
    case 4: // 重置以确保更改。立即重新初始化可能会导致问题。
      if (int_value)
      {
        settings.flags |= BITFLAG_INVERT_ST_ENABLE;
      }
      else
      {
        settings.flags &= ~BITFLAG_INVERT_ST_ENABLE;
      }
      break;
    case 5: // 重置以确保更改。立即重新初始化可能会导致问题。
      if (int_value)
      {
        settings.flags |= BITFLAG_INVERT_LIMIT_PINS;
      }
      else
      {
        settings.flags &= ~BITFLAG_INVERT_LIMIT_PINS;
      }
      break;
    case 6: // 重置以确保更改。立即重新初始化可能会导致问题。
      if (int_value)
      {
        settings.flags |= BITFLAG_INVERT_PROBE_PIN;
      }
      else
      {
        settings.flags &= ~BITFLAG_INVERT_PROBE_PIN;
      }
      probe_configure_invert_mask(false);
      break;
    case 10:
      settings.status_report_mask = int_value;
      break;
    case 11:
      settings.junction_deviation = value;
      break;
    case 12:
      settings.arc_tolerance = value;
      break;
    case 13:
      if (int_value)
      {
        settings.flags |= BITFLAG_REPORT_INCHES;
      }
      else
      {
        settings.flags &= ~BITFLAG_REPORT_INCHES;
      }
      system_flag_wco_change(); // 确保 WCO 立即更新。
      break;
    case 20:
      if (int_value)
      {
        if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE))
        {
          return (STATUS_SOFT_LIMIT_ERROR);
        }
        settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
      }
      else
      {
        settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE;
      }
      break;
    case 21:
      if (int_value)
      {
        settings.flags |= BITFLAG_HARD_LIMIT_ENABLE;
      }
      else
      {
        settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE;
      }
      limits_init(); // 重新初始化以立即更改。注意：有好处但以后可能会有问题。
      break;
    case 22:
      if (int_value)
      {
        settings.flags |= BITFLAG_HOMING_ENABLE;
      }
      else
      {
        settings.flags &= ~BITFLAG_HOMING_ENABLE;
        settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE; // 强制禁用软限制。
      }
      break;
    case 23:
      settings.homing_dir_mask = int_value;
      break;
    case 24:
      settings.homing_feed_rate = value;
      break;
    case 25:
      settings.homing_seek_rate = value;
      break;
    case 26:
      settings.homing_debounce_delay = int_value;
      break;
    case 27:
      settings.homing_pulloff = value;
      break;
    case 30:
      settings.rpm_max = value;
      spindle_init();
      break; // 重新初始化主轴转速校准
    case 31:
      settings.rpm_min = value;
      spindle_init();
      break; // 重新初始化主轴转速校准
    case 32:
      if (int_value)
      {
        settings.flags |= BITFLAG_LASER_MODE;
      }
      else
      {
        settings.flags &= ~BITFLAG_LASER_MODE;
      }
      break;
    default:
      return (STATUS_INVALID_STATEMENT);
    }
  }
  write_global_settings();
  return (STATUS_OK);
}

// 初始化配置子系统
void settings_init()
{
  if (!read_global_settings())
  {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_restore(SETTINGS_RESTORE_ALL); // 强制恢复所有 EEPROM 数据。
    report_grbl_settings();
  }
}

// 根据 Grbl 内部轴索引返回步进引脚掩码。
uint8_t get_step_pin_mask(uint8_t axis_idx)
{
  if (axis_idx == X_AXIS)
  {
    return ((1 << X_STEP_BIT));
  }
  if (axis_idx == Y_AXIS)
  {
    return ((1 << Y_STEP_BIT));
  }
#if defined(A_AXIS) || defined(B_AXIS) || defined(C_AXIS) || defined(D_AXIS)
  if (axis_idx == Z_AXIS)
  {
    return ((1 << Z_STEP_BIT));
  }
#endif
#ifdef A_AXIS
  if (axis_idx == A_AXIS)
  {
    return ((1 << A_STEP_BIT));
  }
#endif
#ifdef B_AXIS
  if (axis_idx == B_AXIS)
  {
    return ((1 << B_STEP_BIT));
  }
#endif
#ifdef C_AXIS
  if (axis_idx == C_AXIS)
  {
    return ((1 << C_STEP_BIT));
  }
#endif
#ifdef D_AXIS
  return ((1 << D_STEP_BIT));
#else
  return ((1 << Z_STEP_BIT));
#endif
}

// 根据 Grbl 内部轴索引返回方向引脚掩码。
uint8_t get_direction_pin_mask(uint8_t axis_idx)
{
  if (axis_idx == X_AXIS)
  {
    return ((1 << X_DIRECTION_BIT));
  }
  if (axis_idx == Y_AXIS)
  {
    return ((1 << Y_DIRECTION_BIT));
  }
#if defined(A_AXIS) || defined(B_AXIS) || defined(C_AXIS) || defined(D_AXIS)
  if (axis_idx == Z_AXIS)
  {
    return ((1 << Z_DIRECTION_BIT));
  }
#endif
#ifdef A_AXIS
  if (axis_idx == A_AXIS)
  {
    return ((1 << A_DIRECTION_BIT));
  }
#endif
#ifdef B_AXIS
  if (axis_idx == B_AXIS)
  {
    return ((1 << B_DIRECTION_BIT));
  }
#endif
#ifdef C_AXIS
  if (axis_idx == C_AXIS)
  {
    return ((1 << C_DIRECTION_BIT));
  }
#endif
#ifdef D_AXIS
  return ((1 << D_DIRECTION_BIT));
#else
  return ((1 << Z_DIRECTION_BIT));
#endif
}

// 根据 Grbl 内部轴索引返回限位引脚掩码。
uint8_t get_limit_pin_mask(uint8_t axis_idx)
{
  if (axis_idx == X_AXIS)
  {
    return ((1 << X_LIMIT_BIT));
  }
  if (axis_idx == Y_AXIS)
  {
    return ((1 << Y_LIMIT_BIT));
  }
#if defined(A_AXIS) || defined(B_AXIS) || defined(C_AXIS) || defined(D_AXIS)
  if (axis_idx == Z_AXIS)
  {
    return ((1 << Z_LIMIT_BIT));
  }
#endif
#ifdef A_AXIS
  if (axis_idx == A_AXIS)
  {
    return ((1 << A_LIMIT_BIT));
  }
#endif
#ifdef B_AXIS
  if (axis_idx == B_AXIS)
  {
    return ((1 << B_LIMIT_BIT));
  }
#endif
#ifdef C_AXIS
  if (axis_idx == C_AXIS)
  {
    return ((1 << C_LIMIT_BIT));
  }
#endif
#ifdef D_AXIS
  return ((1 << D_LIMIT_BIT));
#else
  return ((1 << Z_LIMIT_BIT));
#endif
}
