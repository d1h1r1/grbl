/*
  cpu_map.h - CPU 和引脚映射配置文件
  Grbl的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl是一个自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款，重新发布和/或修改
  该软件，版本3或（根据您的选择）任何更新的版本。

  Grbl的发布是希望它能有用，
  但没有任何保证；甚至不含有
  适销性或特定用途的隐含保证。有关更多详细信息，请参见GNU通用公共许可证。

  您应该已随Grbl一起收到GNU通用公共许可证的副本。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

/* cpu_map.h文件作为不同
   处理器类型或替代引脚布局的中央引脚映射选择文件。此版本的Grbl仅支持
   Arduino Mega2560。 */

#ifndef cpu_map_h
#define cpu_map_h

#ifdef CPU_MAP_2560_INITIAL // (Arduino Mega 2560) Working @EliteEng

// 串口中断
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

// 定义步进脉冲输出引脚。注意：所有步进位引脚必须在同一端口上。
#define STEP_DDR DDRA
#define STEP_PORT PORTA
#define STEP_PIN PINA
#define D_STEP_BIT 1                                                                                                                                          // MEGA2560 Digital Pin 23
#define X_STEP_BIT 2                                                                                                                                          // MEGA2560 Digital Pin 24
#define Y_STEP_BIT 3                                                                                                                                          // MEGA2560 Digital Pin 25
#define Z_STEP_BIT 4                                                                                                                                          // MEGA2560 Digital Pin 26
#define A_STEP_BIT 5                                                                                                                                          // MEGA2560 Digital Pin 27
#define B_STEP_BIT 6                                                                                                                                          // MEGA2560 Digital Pin 28
#define C_STEP_BIT 7                                                                                                                                          // MEGA2560 Digital Pin 29
#define STEP_MASK ((1 << D_STEP_BIT) | (1 << X_STEP_BIT) | (1 << Y_STEP_BIT) | (1 << Z_STEP_BIT) | (1 << A_STEP_BIT) | (1 << B_STEP_BIT) | (1 << C_STEP_BIT)) // All step bits

// 定义步进方向输出引脚。注意：所有方向引脚必须在同一端口上。
#define DIRECTION_DDR DDRC
#define DIRECTION_PORT PORTC
#define DIRECTION_PIN PINC
#define X_DIRECTION_BIT 7                                                                                                                                                                             // MEGA2560 Digital Pin 30
#define Y_DIRECTION_BIT 6                                                                                                                                                                             // MEGA2560 Digital Pin 31
#define Z_DIRECTION_BIT 5                                                                                                                                                                             // MEGA2560 Digital Pin 32
#define A_DIRECTION_BIT 4                                                                                                                                                                             // MEGA2560 Digital Pin 33
#define B_DIRECTION_BIT 3                                                                                                                                                                             // MEGA2560 Digital Pin 34
#define C_DIRECTION_BIT 2                                                                                                                                                                             // MEGA2560 Digital Pin 35
#define D_DIRECTION_BIT 1                                                                                                                                                                             // MEGA2560 Digital Pin 36
#define DIRECTION_MASK ((1 << X_DIRECTION_BIT) | (1 << Y_DIRECTION_BIT) | (1 << Z_DIRECTION_BIT) | (1 << A_DIRECTION_BIT) | (1 << B_DIRECTION_BIT) | (1 << C_DIRECTION_BIT) | (1 << D_DIRECTION_BIT)) // All direction bits


// 定义归位/硬限位开关输入引脚和限位中断向量。
// 注意：所有限位位引脚必须在同一端口上。
#define LIMIT_DDR DDRB
#define LIMIT_PORT PORTB
#define LIMIT_PIN PINB
#define X_LIMIT_BIT 4   // MEGA2560 Digital Pin 10
#define Y_LIMIT_BIT 5   // MEGA2560 Digital Pin 11
#define Z_LIMIT_BIT 6   // MEGA2560 Digital Pin 12
#define B_LIMIT_BIT 1   // MEGA2560 Digital Pin 52
#define C_LIMIT_BIT 2   // MEGA2560 Digital Pin 51
#define D_LIMIT_BIT 3   // MEGA2560 Digital Pin 50
#define LIMIT_INT PCIE0 // 引脚更改中断使能引脚
#define LIMIT_INT_vect PCINT0_vect
#define LIMIT_PCMSK PCMSK0                                                                                                                                            // 引脚更改中断寄存器
#define LIMIT_MASK ((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT) | (1 << B_LIMIT_BIT) | (1 << C_LIMIT_BIT) | (1 << D_LIMIT_BIT) | (1) | (1 << 7)) // All limit bits

#define A_LIMIT_DDR DDRK
#define A_LIMIT_PORT PORTK
#define A_LIMIT_PIN PINK
#define A_LIMIT_BIT 2

// 定义用户控制（CONTROL）输入引脚（循环开始、重置、进给暂停）。
// 注意：所有CONTROL引脚必须在同一端口上，并且不能与其他输入引脚（限位）在同一端口上。
#define CONTROL_DDR DDRK
#define CONTROL_PIN PINK
#define CONTROL_PORT PORTK
#define BRUSH_DETECT_BIT 0       // MEGA2560 Analog Pin 8
#define CONTROL_SAFETY_DOOR_BIT 3 // MEGA2560 Analog Pin 11
#define X_ALARM_BIT 4 // MEGA2560 Analog Pin 11
#define Y_ALARM_BIT 5 // MEGA2560 Analog Pin 11
#define Z_ALARM_BIT 6 // MEGA2560 Analog Pin 11
#define STOP_ALARM_BIT 7 // MEGA2560 Analog Pin 11

#define CONTROL_INT PCIE2         // 引脚更改中断使能引脚
#define CONTROL_INT_vect PCINT2_vect
#define CONTROL_PCMSK PCMSK2 // 引脚更改中断寄存器
// #define CONTROL_MASK ((1 << BRUSH_DETECT_BIT) | (1 << CONTROL_SAFETY_DOOR_BIT) | (1 << X_ALARM_BIT) | (1 << Y_ALARM_BIT) | (1 << Z_ALARM_BIT) | (1 << STOP_ALARM_BIT))
#define CONTROL_MASK 0

// 定义探针开关输入引脚。
#define PROBE_DDR DDRL
#define PROBE_PIN PINL
#define PROBE_PORT PORTL
#define PROBE_BIT1 2 // MEGA2560 Digital Pin 14
#define PROBE_BIT0 3 // MEGA2560 Digital Pin 15
#define PROBE_MASK ((1 << PROBE_BIT0) | (1 << PROBE_BIT1))

// 高级配置（下面的变量一般不需要修改）
// 设置定时器以使用TIMER4B，连接到数字引脚7
#define SPINDLE_PWM_MAX_VALUE 1024.0 // 在1/8预分频下，约转换为1.9 kHz的PWM频率
#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE 1 // 必须大于零。
#endif
#define SPINDLE_PWM_OFF_VALUE 0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE - SPINDLE_PWM_MIN_VALUE)
#define SPINDLE_TCCRA_REGISTER TCCR4A
#define SPINDLE_TCCRB_REGISTER TCCR4B
#define SPINDLE_OCR_REGISTER OCR4B
#define SPINDLE_COMB_BIT COM4B1

// 1/8 预分频，16位快速PWM模式
#define SPINDLE_TCCRA_INIT_MASK ((1 << WGM40) | (1 << WGM41))
#define SPINDLE_TCCRB_INIT_MASK ((1 << WGM42) | (1 << WGM43) | (1 << CS41))
#define SPINDLE_OCRA_REGISTER OCR4A   // 16位快速PWM模式需要将顶部复位值存储在此处。
#define SPINDLE_OCRA_TOP_VALUE 0x0400 // PWM计数器复位值。应与PWM_MAX_VALUE的十六进制值相同。
                                      // 定义主轴输出引脚。
#define SPINDLE_PWM_DDR DDRH
#define SPINDLE_PWM_PORT PORTH
#define SPINDLE_PWM_BIT 4 // MEGA2560 Digital Pin 7

#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  // 对于自定义引脚映射或不同的处理器，请复制并编辑可用的cpu
  // 映射文件之一，并根据需要进行修改。确保在
  // config.h文件中也更改定义的名称。
#endif
*/

#endif
