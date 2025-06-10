/*
  grbl.h - Grbl 的主包含文件
  Grbl 的一部分

  版权所有 (c) 2015-2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl 是自由软件：你可以在自由软件基金会发布的 GNU 通用公共许可证条款下重新分发和/或修改
  它，许可证版本为 3，或（根据你的选择）任何更高版本。

  Grbl 的发布是为了希望它能有用，
  但不提供任何担保；甚至没有关于
  适销性或适用于特定目的的隐含担保。有关详细信息，请参见
  GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

#ifndef grbl_h
#define grbl_h

// Grbl 版本控制系统
#define GRBL_VERSION "1.1e"
#define GRBL_VERSION_BUILD "20170114"

// 定义 Grbl 使用的标准库。
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// 定义 Grbl 系统包含文件。注意：请勿更改组织结构。
#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "system.h"
#include "defaults.h"
#include "cpu_map.h"
#include "planner.h"
#include "eeprom.h"
#include "gcode.h"
#include "limits.h"
#include "motion_control.h"
#include "planner.h"
#include "print.h"
#include "probe.h"
#include "protocol.h"
#include "report.h"
#include "serial.h"
#include "spindle_control.h"
#include "stepper.h"
#include "jog.h"
#include "sleep.h"
#include "getTemp.h"
#include "switch.h"
#include "probe_control.h"
#include "rfid_control.h"
#include "tool_change.h"
#include "led.h"

// ---------------------------------------------------------------------------------------
// 编译时对定义值进行错误检查：

#ifndef HOMING_CYCLE_0
  #error "缺少必需的 HOMING_CYCLE_0 定义。"
#endif

#if defined(PARKING_ENABLE)
  #if defined(HOMING_FORCE_SET_ORIGIN)
    #error "当前不支持与 PARKING_ENABLE 一起使用 HOMING_FORCE_SET_ORIGIN。"
  #endif
#endif

#if defined(SPINDLE_PWM_MIN_VALUE)
  #if !(SPINDLE_PWM_MIN_VALUE > 0)
    #error "SPINDLE_PWM_MIN_VALUE 必须大于零。"
  #endif
#endif

#if (REPORT_WCO_REFRESH_BUSY_COUNT < REPORT_WCO_REFRESH_IDLE_COUNT)
  #error "WCO 繁忙刷新少于空闲刷新。"
#endif
#if (REPORT_OVR_REFRESH_BUSY_COUNT < REPORT_OVR_REFRESH_IDLE_COUNT)
  #error "重载繁忙刷新少于空闲刷新。"
#endif
#if (REPORT_WCO_REFRESH_IDLE_COUNT < 2)
  #error "WCO 刷新必须大于 1。"
#endif
#if (REPORT_OVR_REFRESH_IDLE_COUNT < 1)
  #error "重载刷新必须大于 0。"
#endif
// ---------------------------------------------------------------------------------------

#endif
