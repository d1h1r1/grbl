/*
  defaults.h - 默认设置配置文件
  Grbl的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl是一个自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款，重新发布和/或修改
  该软件，版本3或（根据您的选择）任何更新的版本。

  Grbl的发布是希望它能有用，
  但没有任何保证；甚至不含有
  适销性或特定用途的隐含保证。有关更多详细信息，请参见GNU通用公共许可证。

  您应该已随Grbl一起收到GNU通用公共许可证的副本。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

/* defaults.h文件作为不同机器类型（从DIY CNC铣床到现成机器的CNC改造）的中央默认设置选择器。
   这里列出的设置文件由用户提供，因此您的结果可能会有所不同。不过，这应该
   为您熟悉机器并调整设置以满足您的需求提供一个良好的起点。
   注意：确保在config.h中仅定义一个DEFAULTS_XXX值。 */


#ifndef defaults_h

#ifdef DEFAULTS_GENERIC
  // Grbl通用默认设置。应适用于不同的机器。
  #define DEFAULT_X_STEPS_PER_MM 250.0
  #define DEFAULT_Y_STEPS_PER_MM 250.0
  #define DEFAULT_Z_STEPS_PER_MM 250.0
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHERLINE_5400
  // 描述：Sherline 5400铣床配备三台NEMA 23 Keling KL23H256-21-8B 185 oz-in步进电机，
// 由三台Pololu A4988步进驱动器驱动，使用30V、6A电源，每个绕组1.5A。
  #define MICROSTEPS 2
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV (0.050*MM_PER_INCH) // 0.050 inch/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 635.0 // mm/min (25 ipm)
  #define DEFAULT_Y_MAX_RATE 635.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 635.0 // mm/min
  #define DEFAULT_X_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 225.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 125.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 170.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 2800.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm  
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))  
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 50.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 635.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO
// 描述：Shapeoko CNC铣床配备三台NEMA 17步进电机，由Synthetos grblShield驱动，使用24V、4.2A电源。
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 400
  #define MM_PER_REV_XY (0.08*18*MM_PER_INCH) // 0.08 in belt pitch, 18 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 400
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))  
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm  
#endif

#ifdef DEFAULTS_SHAPEOKO_2
// 描述：Shapeoko CNC铣床配备三台NEMA 17步进电机，由Synthetos grblShield驱动，使用28V电源。
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (250.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (250.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 290.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 290.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm  
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false  
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm  
#endif

#ifdef DEFAULTS_SHAPEOKO_3
  // 描述：Shapeoko CNC铣床配备三台NEMA 23步进电机，由CarbideMotion驱动。
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 8
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 425.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 465.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 80.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.01 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 100.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 1000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 25 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 5.0 // mm  
#endif

#ifdef DEFAULTS_X_CARVE_500MM
// 描述：X-Carve 3D雕刻机CNC铣床配备三台200步/转电机，由Synthetos grblShield驱动，使用24V电源。
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 2.117 // ACME 3/8-12 Leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 290.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 290.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm  
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 750.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm 
#endif

#ifdef DEFAULTS_X_CARVE_1000MM
// 描述：X-Carve 3D雕刻机CNC铣床配备三台200步/转电机，由Synthetos grblShield驱动，使用24V电源。
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 2.117 // ACME 3/8-12 Leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 740.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 790.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm  
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 750.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm 
#endif

#ifdef DEFAULTS_ZEN_TOOLWORKS_7x7
// 描述：Zen Toolworks 7x7铣床配备三台Shinano SST43D2121 65oz-in NEMA 17步进电机。
// 导螺杆与一些ZT W套件不同，后者大多数为1.25mm/转，而这里为8.0mm/转。
// 由30V、6A电源和TI DRV8811步进电机驱动器驱动。
  #define MICROSTEPS 8
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV 8.0 // 8 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 190.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 180.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 150.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm  
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false  
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_OXCNC
// Grbl设置用于OpenBuilds OX CNC机器
// http://www.openbuilds.com/builds/openbuilds-ox-cnc-machine.341/
  #define DEFAULT_X_STEPS_PER_MM 26.670
  #define DEFAULT_Y_STEPS_PER_MM 26.670
  #define DEFAULT_Z_STEPS_PER_MM 50
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 500.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 750.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 80.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SIMULATOR
// 仅用于Grbl模拟器（www.github.com/grbl/grbl-sim）
// Grbl通用默认设置。应适用于不同的机器。
  #define DEFAULT_X_STEPS_PER_MM 1000.0
  #define DEFAULT_Y_STEPS_PER_MM 1000.0
  #define DEFAULT_Z_STEPS_PER_MM 1000.0
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 1000.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 1000.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 1000.0 // mm
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_ABC_AXIS
  #define DEFAULT_A_STEPS_PER_MM 250.0
  #define DEFAULT_A_MAX_RATE 500.0 // mm/min
  #define DEFAULT_A_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_A_MAX_TRAVEL 200.0 // mm

  #define DEFAULT_B_STEPS_PER_MM 250.0
  #define DEFAULT_B_MAX_RATE 500.0 // mm/min
  #define DEFAULT_B_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_B_MAX_TRAVEL 200.0 // mm

  #define DEFAULT_C_STEPS_PER_MM 250.0
  #define DEFAULT_C_MAX_RATE 500.0 // mm/min
  #define DEFAULT_C_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_C_MAX_TRAVEL 200.0 // mm
#endif

#endif
