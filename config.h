/*
  config.h - 编译时配置
  Grbl的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改它，不论是版本3的许可证，还是（根据您的选择）任何更高版本。

  Grbl的发布是为了希望它能对您有用，但不提供任何担保，甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本，随Grbl一起提供。如果没有，请访问<http://www.gnu.org/licenses/>。
*/

// 此文件包含Grbl内部系统的编译时配置。对于大多数用户来说，通常不需要直接修改这些设置，但它们用于特定需求，例如性能调优或调整不典型的机器。

// 重要提示：任何更改都需要对源代码进行完整的重新编译才能生效。

#ifndef config_h
#define config_h
#include "grbl.h" // 为了与Arduino IDE的兼容性。

// 定义CPU引脚映射和默认设置。
// 注意：OEM可以通过将其特定的默认值和引脚映射放在此文件的底部，避免维护/更新defaults.h和cpu_map.h文件，仅使用一个配置文件。
// 如果这样做，只需注释掉这两个定义，并参见下面的说明。

#define DEFAULTS_GENERIC
#define DEFAULTS_ABC_AXIS
#define CPU_MAP_2560_INITIAL

// 串口波特率
// #define BAUD_RATE 256000
#define BAUD_RATE 115200

// 定义实时命令特殊字符。这些字符是直接从串行读取数据流中“挑选”出来的，且不会传递给grbl行执行解析器。
// 选择不在流式G代码程序中存在的字符，并且绝不能存在的字符。如果用户设置可用，可以使用ASCII控制字符。
// 此外，可以选择扩展ASCII代码（>127），这些代码在G代码程序中从不出现，用于接口程序。
// 注意：如果更改，请手动更新report.c中的帮助信息。

#define CMD_RESET 0x18 // ctrl-x.
#define CMD_STATUS_REPORT '?'
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'

// 注意：所有覆盖的实时命令必须在扩展ASCII字符集中，从字符值128（0x80）开始，直到255（0xFF）。
// 如果将正常的实时命令，如状态报告、进给保持、复位和循环开始，
// 移动到扩展集空间，serial.c中的RX ISR将需要修改以适应该更改。

// #define CMD_RESET 0x80
// #define CMD_STATUS_REPORT 0x81
// #define CMD_CYCLE_START 0x82
// #define CMD_FEED_HOLD 0x83
#define CMD_SAFETY_DOOR 0x84
#define CMD_JOG_CANCEL 0x85
#define CMD_DEBUG_REPORT 0x86   // 仅在启用DEBUG时，发送调试报告，格式为'{}'。
#define CMD_FEED_OVR_RESET 0x90 // 将进给倍率恢复为100%。
#define CMD_FEED_OVR_COARSE_PLUS 0x91
#define CMD_FEED_OVR_COARSE_MINUS 0x92
#define CMD_FEED_OVR_FINE_PLUS 0x93
#define CMD_FEED_OVR_FINE_MINUS 0x94
#define CMD_RAPID_OVR_RESET 0x95 // 将快速倍率恢复为100%。
#define CMD_RAPID_OVR_MEDIUM 0x96
#define CMD_RAPID_OVR_LOW 0x97
// #define CMD_RAPID_OVR_EXTRA_LOW 0x98  // *不支持*
#define CMD_SPINDLE_OVR_RESET 0x99 // 将主轴倍率恢复为100%。
#define CMD_SPINDLE_OVR_COARSE_PLUS 0x9A
#define CMD_SPINDLE_OVR_COARSE_MINUS 0x9B
#define CMD_SPINDLE_OVR_FINE_PLUS 0x9C
#define CMD_SPINDLE_OVR_FINE_MINUS 0x9D
#define CMD_SPINDLE_OVR_STOP 0x9E
#define CMD_COOLANT_FLOOD_OVR_TOGGLE 0xA0
#define CMD_COOLANT_MIST_OVR_TOGGLE 0xA1

// 如果启用了归位，归位初始化锁会在开机时将Grbl置于报警状态。这迫使用户在执行其他任何操作之前进行归位循环（或覆盖锁）。
// 这主要是一个安全特性，用于提醒用户进行归位，因为Grbl的位置信息是未知的。

#define HOMING_INIT_LOCK // 注释以禁用

// 定义归位循环模式的位掩码。归位循环首先执行搜索模式，以快速接触限位开关，然后进入较慢的定位模式，最后进行短暂的回拉运动以脱离限位开关。
// 以下HOMING_CYCLE_x定义按顺序执行，从后缀0开始，仅完成指定轴的归位例程。如果某个轴在定义中被省略，它将不会归位，也不会更新系统的位置。
// 这意味着，这允许使用非标准的笛卡尔机器的用户，例如车床（先X再Z，没有Y），根据需要配置归位循环行为。
// 注意：归位循环设计允许共享限位引脚，如果轴不在同一循环中，但这需要在cpu_map.h文件中进行一些引脚设置更改。
// 例如，默认的归位循环可以与X或Y限位引脚共享Z限位引脚，因为它们在不同的循环中。通过共享一个引脚，可以释放一个宝贵的IO引脚供其他用途。
// 理论上，如果所有轴以单独的循环归位，所有轴的限位引脚可以减少到一个引脚，或者相反，所有三个轴在不同的引脚上，但在一个循环中归位。此外，需要注意的是，硬限位的功能不会受引脚共享的影响。
// 注意：默认设置适用于传统的三轴CNC机器。首先清除Z轴，然后是X和Y。

#define HOMING_CYCLE_0 (1 << Z_AXIS)                   // 必须：首先移动Z轴以清除工作空间。
#define HOMING_CYCLE_1 ((1 << X_AXIS) | (1 << Y_AXIS)) // 可选：然后同时移动X轴和Y轴。
// #define HOMING_CYCLE_2                         // 可选：取消注释并添加轴掩码以启用
// #define HOMING_CYCLE_3                         // 可选：取消注释并添加轴掩码以启用
// #define HOMING_CYCLE_4                         // 可选：取消注释并添加轴掩码以启用
// #define HOMING_CYCLE_5                         // 可选：取消注释并添加轴掩码以启用

// 注意：以下是为2轴机器设置归位的两个示例。
// #define HOMING_CYCLE_0 ((1<<X_AXIS)|(1<<Y_AXIS))    // 不兼容COREXY：在一个循环中同时归位X和Y。

// #define HOMING_CYCLE_0 (1<<X_AXIS)  // 兼容COREXY：首先归位X
// #define HOMING_CYCLE_1 (1<<Y_AXIS)  // 兼容COREXY：然后归位Y

// 在机器最初移动到限位开关后执行的归位循环次数。
// 这有助于防止超调，并应提高重复性。此值应为1或更大。

#define N_HOMING_LOCATE_CYCLE 1 // Integer (1-128)

// 启用单轴归位命令。$HX、$HY和$HZ分别用于X、Y和Z轴的归位。完整的归位循环仍然可以通过$H命令调用。默认情况下禁用。此选项仅用于满足需要在二轴和三轴机器之间切换的用户。
// 这实际上是非常少见的。如果您有一台二轴机器，请勿使用此选项。相反，只需修改二轴的归位循环。

// #define HOMING_SINGLE_AXIS_COMMANDS  // 默认禁用。取消注释以启用。

// 归位后，Grbl默认会将整个机器空间设置为负空间，这在专业CNC机器中很常见，无论限位开关位于何处。取消注释此定义以强制Grbl始终在归位位置设置机器原点，而不考虑开关的方向。

#define HOMING_FORCE_SET_ORIGIN // 取消注释以启用。

// Grbl启动时执行的块数。这些块存储在EEPROM中，大小和地址在settings.h中定义。根据当前设置，最多可以存储和按顺序执行2个启动块。这些启动块通常用于根据用户偏好设置G代码解析器的状态。

#define N_STARTUP_LINE 2 // Integer (1-2)

// Grbl为某些值类型打印的浮动小数点数。这些设置由CNC机器中实际观察到的常见值决定。例如，位置值不能小于0.001mm或0.0001in，因为机器在物理上无法更精确。
// 因此，通常不需要更改这些设置，但如果需要，您可以在这里进行更改。
// 注意：必须是0到约4的整数值。超过4可能会出现舍入错误。

#define N_DECIMAL_COORDVALUE_INCH 4 // 以英寸为单位的坐标或位置值
#define N_DECIMAL_COORDVALUE_MM 3   // 以毫米为单位的坐标或位置值
#define N_DECIMAL_RATEVALUE_INCH 1  // 以英寸/分钟为单位的速率或速度值
#define N_DECIMAL_RATEVALUE_MM 0    // 以毫米/分钟为单位的速率或速度值
#define N_DECIMAL_SETTINGVALUE 3    // 浮点设置值的小数位数
#define N_DECIMAL_RPMVALUE 0        // 以每分钟转数（RPM）为单位的值

// 如果您的机器有两个限位开关并联连接到一个轴，您需要启用此功能。由于两个开关共享一个引脚，Grbl无法判断哪个开关被启用。
// 此选项仅影响归位，当限位被触发时，Grbl会发出报警并强制用户手动解除限位开关。否则，如果您每个轴都有一个限位开关，请不要启用此选项。
// 保持禁用状态，您可以在限位开关上执行归位循环，而无需将机器移开。

// #define LIMITS_TWO_SWITCHES_ON_AXES

// 在成功的探测循环后，此选项会通过自动生成的消息立即提供探测坐标的反馈。如果禁用，用户仍然可以通过Grbl的'$#'打印参数访问最后的探测坐标。

#define MESSAGE_PROBE_COORDINATES // 默认启用。注释以禁用。

// 此选项使进给保持输入充当安全门开关。安全门触发时，会立即强制进给保持，然后安全地切断机器电源。恢复操作在安全门重新闭合之前被阻止。
// 当安全门闭合时，Grbl将重新供电，并在先前的工具路径上恢复操作，就像什么都没发生过一样。

// #define ENABLE_SAFETY_DOOR_INPUT_PIN  // 默认禁用。取消注释以启用。

// 在安全门开关被切换并恢复后，此设置设置恢复主轴和冷却液之间的电源延迟。

#define SAFETY_DOOR_SPINDLE_DELAY 4.0 // Float (seconds)
#define SAFETY_DOOR_COOLANT_DELAY 1.0 // Float (seconds)

// 启用 CoreXY 运动学。仅与 CoreXY 机器一起使用。
// 重要提示：如果启用了归位功能，您必须重新配置上面的归位循环 #defines 为
// #define HOMING_CYCLE_0 (1<<X_AXIS) 和 #define HOMING_CYCLE_1 (1<<Y_AXIS)
// 注意：此配置选项改变了 X 轴和 Y 轴的运动原理，具体定义在 (http://corexy.com/theory.html)。
// 假定电机的位置和接线与描述完全一致，否则运动可能会出现奇怪的方向。
// Grbl 要求 CoreXY A 和 B 电机在内部具有相同的每毫米步数。

// #define COREXY  // 默认禁用。取消注释以启用。

// 根据掩码反转控制命令引脚的逻辑。这意味着您可以在指定引脚上使用常闭开关，而不是默认的常开开关。
// 注意：顶部选项将掩码并反转所有控制引脚。底部选项是反转只有两个控制引脚的示例，即安全门和复位。请参阅 cpu_map.h 以获取其他位定义。

// #define INVERT_CONTROL_PIN_MASK CONTROL_MASK  // 默认禁用。取消注释以启用。
// #define INVERT_CONTROL_PIN_MASK ((1<<CONTROL_SAFETY_DOOR_BIT)|(CONTROL_RESET_BIT)) // 默认禁用。

// 根据以下掩码反转选择的限位引脚状态。这会影响所有限位引脚功能，
// 如硬限位和回归原点。然而，这与整体反转限位设置不同。
// 此构建选项仅反转此处定义的限位引脚，然后反转限位设置将应用于所有引脚。
// 当用户的机器上安装了混合类型的限位引脚，包括常开（NO）和常闭（NC）开关时，这很有用。
// 注意：请不要使用此选项，除非您有必要的情况。

// #define INVERT_LIMIT_PIN_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT))  // 默认禁用。取消注释以启用。

// 将主轴使能引脚的逻辑从低禁用/高启用反转为低启用/高禁用。
// 对于某些预构建的电子板很有用。

// #define INVERT_SPINDLE_ENABLE_PIN  // 默认禁用。取消注释以启用。

// 将选定的冷却引脚的逻辑从低禁用/高启用反转为低启用/高禁用。
// 对于某些预构建的电子板很有用。

// #define INVERT_COOLANT_FLOOD_PIN // 默认禁用。取消注释以启用。
// #define INVERT_COOLANT_MIST_PIN // 默认禁用。注意：在 config.h 中启用 M7 雾状冷却。

// 当 Grbl 断电循环或通过 Arduino 重置按钮进行硬重置时，Grbl 默认以无警报（ALARM）状态启动。
// 这样可以让新用户尽可能简单地开始使用 Grbl。当启用归位（homing）并且用户安装了限位开关时，
// Grbl 将以警报状态启动，以表示 Grbl 不知道其位置，并强制用户在继续之前进行归位。
// 该选项强制 Grbl 始终初始化为警报状态，无论是否启用归位。
// 此选项主要用于希望此电源循环行为的 OEM 和 LinuxCNC 用户。

// #define FORCE_INITIALIZATION_ALARM // 默认禁用。

// 在上电或重置时，Grbl 将检查限位开关状态，以确保它们未处于活动状态
// 然后再初始化。如果检测到问题并且启用了硬限位设置，Grbl 将
// 简单地向用户发送消息以检查限位，并进入警报状态，而不是闲置。Grbl 不会抛出警报消息。

#define CHECK_LIMITS_AT_INIT

// ---------------------------------------------------------------------------------------
// 高级配置选项：
// 启用调试目的的代码。不是一般用途，并且始终处于不断变化中。
// #define DEBUG // Uncomment to enable. Default disabled. // 取消注释以启用。默认禁用。

// 配置快速、进给和主轴超速设置。这些值定义了最大和最小允许的超速值以及每个命令接收的粗细增量。请注意描述中允许的值。

#define DEFAULT_FEED_OVERRIDE 100         // 100%。请勿更改此值。
#define MAX_FEED_RATE_OVERRIDE 200        // 程序进给速率的百分比（100-255）。通常为120%或200%。
#define MIN_FEED_RATE_OVERRIDE 10         // 程序进给速率的百分比（1-100）。通常为50%或1%。
#define FEED_OVERRIDE_COARSE_INCREMENT 10 // (1-99)。通常为10%。
#define FEED_OVERRIDE_FINE_INCREMENT 1    // (1-99)。通常为1%。

#define DEFAULT_RAPID_OVERRIDE 100 // 100%。请勿更改此值。
#define RAPID_OVERRIDE_MEDIUM 50   // 快速进给的百分比（1-99）。通常为50%。
#define RAPID_OVERRIDE_LOW 25      // 快速进给的百分比（1-99）。通常为25%。
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *不支持* 快速进给的百分比（1-99）。通常为5%。

#define DEFAULT_SPINDLE_SPEED_OVERRIDE 100   // 100%。请勿更改此值。
#define MAX_SPINDLE_SPEED_OVERRIDE 200       // 程序主轴转速的百分比（100-255）。通常为200%。
#define MIN_SPINDLE_SPEED_OVERRIDE 10        // 程序主轴转速的百分比（1-100）。通常为10%。
#define SPINDLE_OVERRIDE_COARSE_INCREMENT 10 // (1-99)。通常为10%。
#define SPINDLE_OVERRIDE_FINE_INCREMENT 1    // (1-99)。通常为1%。

// 当执行M2或M30程序结束命令时，大多数G代码状态会恢复为默认值。
// 此编译时选项包括在程序结束时将进给、快速和主轴转速覆盖值恢复为其默认值。
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END // 默认启用。注释以禁用。

// Grbl v1.1及以后的状态报告更改也删除了禁用/启用大多数数据字段的能力。
// 这对GUI开发者造成了问题，他们必须管理多种场景和配置。
// 新报告样式的效率提高允许在没有潜在性能问题的情况下发送所有数据字段。
// 注意：以下选项仅在特殊情况下提供禁用某些数据字段的方法，但请注意GUI可能依赖于这些数据。
// 如果禁用，可能会导致不兼容。
#define REPORT_FIELD_BUFFER_STATE       // 默认启用。注释以禁用。
#define REPORT_FIELD_PIN_STATE          // 默认启用。注释以禁用。
#define REPORT_FIELD_CURRENT_FEED_SPEED // 默认启用。注释以禁用。
#define REPORT_FIELD_WORK_COORD_OFFSET  // 默认启用。注释以禁用。
#define REPORT_FIELD_OVERRIDES          // 默认启用。注释以禁用。
#define REPORT_FIELD_LINE_NUMBERS       // 默认启用。注释以禁用。

// 一些状态报告数据对于实时情况并不必要，仅间歇性需要，因为值不经常变化。
// 以下宏配置状态报告调用多少次后刷新相关数据并包含在状态报告中。
// 但是，如果这些值之一发生变化，Grbl将自动在下一个状态报告中包含该数据，
// 无论当时计数是多少。这有助于减少高频报告和激进流媒体的通信开销。
// 还有忙碌和空闲刷新计数，这使Grbl在不做重要事情时更频繁地发送刷新。
// 对于良好的GUI，此数据不需要经常刷新，通常在几秒钟内。
// 注意：WCO刷新必须为2或更大。OVR刷新必须为1或更大。
#define REPORT_OVR_REFRESH_BUSY_COUNT 20 // (1-255)
#define REPORT_OVR_REFRESH_IDLE_COUNT 10 // (1-255) 必须小于或等于忙碌计数
#define REPORT_WCO_REFRESH_BUSY_COUNT 30 // (2-255)
#define REPORT_WCO_REFRESH_IDLE_COUNT 10 // (2-255) 必须小于或等于忙碌计数

// 加速度管理子系统的时间分辨率。较高的数值提供更平滑的加速度，
// 在以非常高的进给率运行的机器上特别明显，但可能会对性能产生负面影响。
// 此参数的正确值依赖于机器，因此建议仅将其设置为所需的最大值。
// 近似成功值可以从50到200或更高范围广泛变化。
// 注意：更改此值也会更改步进段缓冲区中段的执行时间。
// 当增加此值时，整体段缓冲区中存储的时间更少，反之亦然。
// 确保增加/减少步进段缓冲区以适应这些更改。
#define ACCELERATION_TICKS_PER_SECOND 100

// 自适应多轴步进平滑（AMASS）是一项先进功能，能够实现其名称所暗示的功能，
// 平滑多轴运动的步进。在低步进频率（10kHz以下）时，此功能能够特别平滑运动，
// 其中多轴运动的混叠可能会导致可听噪声并使机器抖动。甚至在更低的步进频率下，AMASS自适应并提供更好的步进平滑。
// 有关AMASS系统工作原理的更多细节，请参见stepper.c。
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING // 默认启用。注释以禁用。

// 设置作为Grbl设置写入的最大步进速率。此选项在设置模块中启用错误检查，
// 以防止超出此限制的设置值。最大步进速率严格受CPU速度限制，
// 如果使用的不是以16MHz运行的AVR，则会发生变化。
// 注意：目前禁用，如果闪存空间允许，将启用。
// #define MAX_STEP_RATE_HZ 30000 // Hz

// 默认情况下，Grbl将所有输入引脚设置为正常高操作，并启用其内部上拉电阻。
// 这简化了用户的接线，只需将开关连接到接地，
// 尽管建议用户采取额外措施连接低通滤波器以减少引脚检测的电气噪声。
// 如果用户在Grbl设置中反转引脚，则仅会翻转高或低读数表示活动信号的含义。
// 在正常操作中，这意味着用户需要连接一个常开开关，但如果反转，
// 则意味着用户应该连接一个常闭开关。
// 以下选项禁用内部上拉电阻，将引脚设置为正常低操作，
// 开关现在必须连接到Vcc而不是接地。这还翻转了Grbl设置中的引脚反转含义，
// 其中反转设置现在意味着用户应连接常开开关，反之亦然。
// 注意：与此功能相关的所有引脚都被禁用，即XYZ限制引脚，而不是单个轴。
// 警告：禁用上拉电阻后，需要额外的接线和下拉电阻！
// #define DISABLE_LIMIT_PIN_PULL_UP
// #define DISABLE_PROBE_PIN_PULL_UP
// #define DISABLE_CONTROL_PIN_PULL_UP

// 设置工具长度偏移应用于哪个轴。假设主轴始终与所选轴平行，
// 工具朝向负方向。换句话说，正的工具长度偏移值将从当前位置减去。
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // 默认z轴。有效值为X_AXIS、Y_AXIS或Z_AXIS。

// 仅由可变主轴输出使用。启用时强制PWM输出的最小占空比。
// 当主轴禁用时，PWM引脚仍将读取0V。大多数用户无需此选项，
// 但在某些情况下可能有用。此最小PWM设置与主轴rpm最小设置一致，
// 例如，rpm最大与最大PWM相同。如果您需要在禁用的0V和
// 最小rpm设置的PWM之间有更大的电压差，这一点很方便。
// 该差异是每个PWM值0.02V。因此，当最小PWM为1时，仅有0.02伏特分隔启用和禁用。
// 在PWM 5时，这将为0.1V。请记住，随着最小PWM值的增加，您将开始失去PWM分辨率，
// 因为您在总共255个PWM级别上信号不同主轴转速的范围越来越小。
// 注意：使用以下方程计算最小PWM的占空比：（%占空比）=（SPINDLE_PWM_MIN_VALUE / 255）* 100
// #define SPINDLE_PWM_MIN_VALUE 5 // 默认禁用。取消注释以启用。必须大于零。整数（1-255）。

// 启用后，Grbl将回显已接收的行，该行已被预解析（去除空格，字母大写，无注释），
// 并将立即由Grbl执行。在缓冲区溢出时不会发送回显，但应在发送给Grbl的所有正常行中发送。
// 例如，如果用户发送行'g1 x1.032 y2.45 (测试注释)'，Grbl将以'[echo: G1X1.032Y2.45]'的形式回显。
// 注意：仅用于调试目的！回显时会占用宝贵资源，并可能影响性能。
// 如果绝对需要正常操作，则应大大增加串行写入缓冲区，
// 以帮助最小化串行写入协议中的传输等待。
// #define REPORT_ECHO_LINE_RECEIVED // 默认禁用。取消注释以启用。

// 最小规划交汇速度。设置默认最小交汇速度，规划在每个缓冲区块交汇处，
// 除了从静止状态开始和缓冲区结束，这两者始终为零。
// 此值控制机器在没有考虑加速度限制或相邻块线移动方向之间角度的情况下，
// 穿过交汇点的速度。对于无法容忍工具在瞬间停留的机器，即3D打印机或激光切割机非常有用。
// 如果使用，此值不应大于零或是机器工作所需的最小值。
#define MINIMUM_JUNCTION_SPEED 0.0 // (mm/min)

// 设置规划将允许的最小进给速率。任何低于此值的值将设置为此最小值。
// 这还确保计划的运动始终完成，并考虑到任何浮点舍入错误。
// 虽然不推荐，但低于1.0 mm/min的值可能在较小的机器中有效，
// 也许为0.1 mm/min，但根据多种因素，您的成功可能会有所不同。
#define MINIMUM_FEED_RATE 1.0 // (mm/min)

// 小角度近似的弧生成迭代次数，然后进行精确的弧轨迹
// 修正，使用耗费的sin()和cos()计算。
// 如果弧生成的精确度存在问题，可以减少此参数，
// 或者如果弧执行因过多三角函数计算而变得缓慢，则增加此参数。
#define N_ARC_CORRECTION 12 // 整数（1-255）

// G2/3的弧G代码标准本质上是有问题的。
// 基于半径的弧在半圆（π）或完整圆（2π）时存在可怕的数值错误。
// 基于偏移的弧更准确，但在完整圆（2π）时仍存在问题。
// 此定义考虑在命令为完整圆时基于偏移的弧的浮点问题，
// 但因数值舍入和精度问题而被解释为极小的弧（机器epsilon（1.2e-7rad））。
// 此定义值设置机器epsilon截止值，以确定弧是否为完整圆。
// 注意：在调整此值时要非常小心。它应始终大于1.2e-7，但又不要大于此值太多。
// 默认设置应捕获大多数（如果不是全部）完整弧错误情况。
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-5 // 浮点（弧度）

// 在停留期间执行的时间延迟增量。默认值设置为50毫秒，
// 最大延迟时间大约为55分钟，足够满足大多数应用。
// 增加此延迟将线性增加最大停留时间，但也会降低运行时命令执行的响应性，
// 例如状态报告，因为这些操作在每个停留时间步骤之间执行。
// 还要记住，Arduino延迟计时器对于长延迟并不非常准确。
#define DWELL_TIME_STEP 50 // 整数（1-255）（毫秒）

// 创建方向引脚设置与相应步进脉冲之间的延迟，
// 通过创建另一个中断（Timer2 比较）来管理它。
// 主要的 Grbl 中断（Timer1 比较）设置方向引脚，
// 而不是像正常操作中那样立即设置步进引脚。
// Timer2 比较在步进脉冲延迟时间后触发，
// 用于设置步进引脚，而 Timer2 溢出将完成步进脉冲，
// 只是现在延迟了步进脉冲时间加上步进脉冲延迟。
// （感谢 langwadt 的想法！）
// 注意：取消注释以启用。推荐的延迟必须 > 3us，
// 并且与用户提供的步进脉冲时间相加，
// 总时间不得超过 127us。某些设置的报告成功值在 5 到 20us 之间。
// #define STEP_PULSE_DELAY 10 // 步进脉冲延迟，以微秒为单位。默认禁用。

// 规划缓冲区中可以同时规划的线性运动的数量。
// Grbl 使用的大部分 RAM 基于此缓冲区大小。
// 仅在有额外可用 RAM 的情况下增加，比如在为 Mega 或 Sanguino 重新编译时。
// 如果由于缺少可用 RAM 而导致 Arduino 开始崩溃，
// 或者 CPU 在执行时难以跟上新的输入运动的规划，
// 则减少此值。
// #define BLOCK_BUFFER_SIZE 36  // 取消注释以覆盖 planner.h 中的默认值。

// 管理步进执行算法和规划块之间的中介步进段缓冲区的大小。
// 每个段是一组在固定时间内以恒定速度执行的步进，
// 该时间由 ACCELERATION_TICKS_PER_SECOND 定义。
// 它们的计算方式使得规划块速度轮廓得以准确追踪。
// 此缓冲区的大小决定了其他 Grbl 进程计算和执行其操作的步骤执行前置时间，
// 目前约为 ~50 毫秒的步进移动。
// #define SEGMENT_BUFFER_SIZE 10 // 取消注释以覆盖 stepper.h 中的默认值。

// 从串行输入流执行的行缓冲区大小。
// 同时，管理每个启动块的大小，因为它们作为此大小的字符串存储。
// 确保考虑在 settings.h 中定义的内存地址的可用 EEPROM 以及所需的启动块数量。
// 注意：80 个字符在极端情况下不是问题，但行缓冲区大小可能太小，
// g-code 块可能被截断。官方的 g-code 标准支持最多 256 个字符。
// 在未来的版本中，当我们知道可以重新投资多少额外内存时，
// 此默认值将被增加。
// #define LINE_BUFFER_SIZE 256  // 取消注释以覆盖 protocol.h 中的默认值。

// 串行发送和接收缓冲区大小。
// 接收缓冲区通常被用作另一个流缓冲区，以存储传入块，
// 以便 Grbl 在准备好时处理。大多数流接口会字符计数并跟踪每个块发送到每个块响应。
// 因此，如果需要更深的接收缓冲区用于流式传输且可用内存允许，
// 请增加接收缓冲区。发送缓冲区主要处理 Grbl 中的消息。
// 仅在发送大型消息且 Grbl 开始停滞，等待发送消息的其余部分时增加。
// 注意：缓冲区大小值必须大于零且小于 256。
// #define RX_BUFFER_SIZE 255 // 取消注释以覆盖 serial.h 中的默认值
// #define TX_BUFFER_SIZE 255

// 存储在 EEPROM 中的数据字符串的最大行长度。用于启动行和构建信息。
// 此大小与 LINE_BUFFER_SIZE 不同，因为 EEPROM 通常大小有限。
// 注意：更改此值时要非常小心。检查 EEPROM 地址位置以确保这些字符串存储位置不会相互破坏。
// #define EEPROM_LINE_SIZE 80 // 取消注释以覆盖 settings.h 中的默认值。

// 切换串行通信的 XON/XOFF 软件流控制。由于涉及当前 Arduino 上的 Atmega8U2 USB-to-serial 芯片的问题，
// 不正式支持。因为这些芯片上的固件不支持 XON/XOFF 流控制字符，
// 并且芯片中的中间缓冲区会导致延迟和溢出问题，
// 但使用特定编程的 UI 来管理此延迟问题已被确认有效。
// 此外，基于旧的 FTDI FT232RL 的 Arduino（Duemilanove）已知可以与标准终端程序一起工作，
// 因为它们的固件正确管理这些 XON/XOFF 字符。无论如何，请向 Grbl 管理员报告任何成功的案例！
// #define ENABLE_XONXOFF // 默认禁用。取消注释以启用。

// 为硬极限开关提供一个简单的软件去抖动功能。
// 启用时，监控硬极限开关引脚的中断将启用 Arduino 的看门狗定时器，
// 在大约 32 毫秒的延迟后重新检查限位引脚状态。
// 这可以帮助 CNC 机器解决其硬极限开关的虚假触发问题，
// 但它不会修复外部源的信号电缆上的电气干扰问题。
// 建议首先使用屏蔽信号电缆，并将其屏蔽层连接到地面（旧 USB/计算机电缆有效且便宜）
// 并在每个限位引脚中连接一个低通电路。
#define ENABLE_SOFTWARE_DEBOUNCE // 默认禁用。取消注释以启用。

// 配置 Grbl 检查模式下探测周期后的位置信息。
// 禁用时将位置设置为探测目标，启用时将位置设置为起始位置。
// #define SET_CHECK_MODE_PROBE_TO_START // 默认禁用。取消注释以启用。

// 强制 Grbl 在处理器检测到硬限位 ISR 例程内的引脚更改时检查硬限位开关的状态。
// 默认情况下，Grbl 会在任何引脚更改时触发硬限位警报，
// 因为弹跳开关可能导致这样的状态检查错误地读取引脚。
// 当触发硬限位时，它们应该 100% 可靠，因此此选项默认禁用。
// 仅当您的系统/电子设备能够保证开关不会弹跳时，
// 我们建议启用此选项。这将有助于防止在机器从开关 disengage 时触发硬限位。
// 注意：如果启用 SOFTWARE_DEBOUNCE，此选项将无效。
#define HARD_LIMIT_FORCE_STATE_CHECK // 默认禁用。取消注释以启用。

// 调整归位循环搜索和定位标量。
// 这些是 Grbl 的归位循环使用的乘数，
// 以确保限位开关在循环的每个阶段都被接触和清除。
// 搜索阶段使用轴的最大移动设置乘以 SEARCH_SCALAR 来确定查找限位开关的距离。
// 找到后，定位阶段开始，使用归位拉出距离设置乘以 LOCATE_SCALAR 来重新接触限位开关。
// 注意：这两个值都必须大于 1.0，以确保正常功能。
// #define HOMING_AXIS_SEARCH_SCALAR  1.5 // 取消注释以覆盖 limits.c 中的默认值。
// #define HOMING_AXIS_LOCATE_SCALAR  10.0 // 取消注释以覆盖 limits.c 中的默认值。

// 启用 '$RST=*'、'$RST=$' 和 '$RST=#' EEPROM 恢复命令。
// 在某些情况下，这些命令可能不需要。只需注释掉所需的宏以禁用它。
// 注意：请参见 SETTINGS_RESTORE_ALL 宏以自定义 `$RST=*` 命令。
#define ENABLE_RESTORE_EEPROM_WIPE_ALL         // '$RST=*' 默认启用。注释以禁用。
#define ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // '$RST=$' 默认启用。注释以禁用。
#define ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // '$RST=#' 默认启用。注释以禁用。

// 定义在设置版本更改和 `$RST=*` 命令时恢复的 EEPROM 数据。
// 每当设置或其他 EEPROM 数据结构在 Grbl 版本之间发生变化时，Grbl 将自动清除并恢复 EEPROM。
// 此宏控制清除和恢复哪些数据。这对于需要保留某些数据的 OEM 特别有用。
// 例如，可以通过单独的 .INO 草图将 BUILD_INFO 字符串写入 Arduino EEPROM，以包含产品数据。
// 修改此宏以不恢复构建信息 EEPROM 将确保在固件升级后保留该数据。
// 注意：取消注释以覆盖 settings.h 中的默认值
// #define SETTINGS_RESTORE_ALL (SETTINGS_RESTORE_DEFAULTS | SETTINGS_RESTORE_PARAMETERS | SETTINGS_RESTORE_STARTUP_LINES | SETTINGS_RESTORE_BUILD_INFO)

// 启用 '$I=(字符串)' 构建信息写入命令。如果禁用，任何现有的构建信息数据必须
// 通过外部手段以有效的校验和值放入 EEPROM。此宏选项可防止用户覆盖此数据，
// 用于存储 OEM 产品数据。
// 注意：如果禁用并确保 Grbl 永远不会更改构建信息行，您还需要启用
// 上面的 SETTING_RESTORE_ALL 宏并从掩码中移除 SETTINGS_RESTORE_BUILD_INFO。
// 注意：请参见随附的 grblWrite_BuildInfo.ino 示例文件，以单独写入此字符串。
#define ENABLE_BUILD_INFO_WRITE_COMMAND // '$I=' 默认启用。注释以禁用。

// AVR 处理器在 EEPROM 写入期间需要禁用所有中断。这包括
// 步进 ISR 和串行通信 ISR。在长时间的 EEPROM 写入过程中，这种 ISR 暂停可能会
// 导致活动步进丢失位置和串行接收数据丢失。此配置
// 选项强制在写入 EEPROM 时清空规划器缓冲区，以防止
// 丢失步进的任何机会。
// 然而，这并不能防止在 EEPROM 写入期间丢失串行 RX 数据，特别是
// 如果 GUI 同时预先填充串行 RX 缓冲区。强烈建议
// GUI 将这些 gcodes（G10，G28.1，G30.1）标记为始终在包含
// 这些命令的块之后等待 'ok' 再发送更多数据，以消除此问题。
// 注意：大多数 EEPROM 写入命令在作业期间隐式阻止（所有 '$' 命令）。然而，
// 坐标设置 g-code 命令（G10，G28/30.1）不会，因为它们是活动流式
// 作业的一部分。此时，此选项仅强制与这些 g-code 命令同步规划器缓冲区。
#define FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE // 默认启用。注释以禁用。

// 在 Grbl v0.9 及之前，存在一个旧的未解决错误，其中报告的 `WPos:` 工作位置可能与正在执行的内容不符，
// 因为 `WPos:` 是基于 g-code 解析器状态的，该状态可能落后于几个动作。
// 此选项在有命令更改工作坐标偏移 `G10，G43.1，G92，G54-59` 时强制清空、同步并停止
// 规划器缓冲区。这是确保 `WPos:` 始终正确的最简单方法。幸运的是，使用这些命令需要连续运动的情况非常少见。
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // 默认启用。注释以禁用。

// 默认情况下，Grbl 禁用所有 G38.x 探测循环命令的进给速率覆盖。
// 虽然这可能与某些专业级机器控制有所不同，但可以说应该是这样的。
// 大多数探测传感器产生的误差水平依赖于速度。
// 通过将探测循环保持在其编程的进给速率，探测传感器的重复性应该更好。
// 如果需要，可以通过取消注释下面的定义来禁用此行为。
// #define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // 默认禁用。取消注释以启用。

// 启用并配置在安全门状态下的停车运动方法。主要针对
// 希望为其集成机器提供此功能的 OEM。目前，Grbl 假定
// 停车运动仅涉及一个轴，尽管停车实现是为了
// 通过更改停车源代码轻松重构以适应任意数量的运动。
// 此时，Grbl 仅支持停车一个轴（通常是 Z 轴），
// 在缩回时向正方向移动，在恢复位置时向负方向移动。
// 该运动以缓慢的拉出缩回运动、断电和快速停车执行。
// 恢复到恢复位置遵循这些设定运动的反向：快速恢复到
// 拉出位置，通电并有超时，然后以较慢的拉出速率
// 冲击回到原始位置。
// 注意：仍在进行中。机器坐标必须在所有负空间中，并且
// 不与 HOMING_FORCE_SET_ORIGIN 启用一起工作。停车运动也仅在
// 正方向上移动。
// #define PARKING_ENABLE  // 默认禁用。取消注释以启用

// 如果启用，配置停车运动的选项。
#define PARKING_AXIS Z_AXIS           // 定义执行停车运动的轴
#define PARKING_TARGET -5.0           // 停车轴目标。以 mm 为单位，作为机器坐标 [-max_travel,0]。
#define PARKING_RATE 500.0            // 拉出后停车的快速速率，单位为 mm/min。
#define PARKING_PULLOUT_RATE 100.0    // 拉出/冲击的慢进给速率，单位为 mm/min。
#define PARKING_PULLOUT_INCREMENT 5.0 // 主轴拉出和冲击距离，单位为 mm。增量距离。
// 必须为正值或等于零。

// 启用并配置 Grbl 的睡眠模式功能。如果主轴或冷却液被供电且 Grbl
// 不在主动移动或接收任何命令，睡眠计时器将开始。如果收到任何数据或命令，
// 睡眠计时器将重置并重新启动，直到上述条件不再满足。
// 如果睡眠计时器到期，Grbl 将立即执行睡眠模式，通过关闭主轴
// 和冷却液并进入安全睡眠状态。如果停车已启用，Grbl 也将停车。
// 在睡眠模式下，只有硬重置/软重置可以退出，作业将无法恢复。
// 注意：睡眠模式是一个安全特性，主要用于解决通信断开的问题。
// 为了防止 Grbl 进入睡眠状态，可以使用一系列 '?' 状态报告命令作为连接的“心跳”。
// #define SLEEP_ENABLE  // 默认禁用。取消注释以启用。
#define SLEEP_DURATION 5.0 // 在执行睡眠模式之前的秒数（0.25 - 61.0）。

// 此选项将在进给保持期间自动禁用激光，通过在停止后立即调用主轴停止
// 覆盖来实现。然而，这也意味着如果需要，激光仍然可以通过禁用主轴停止覆盖重新启用。
// 这纯粹是一个安全特性，以确保激光在停止时不会意外保持供电而引发火灾。
#define DISABLE_LASER_DURING_HOLD // 默认启用。注释以禁用。

/* ---------------------------------------------------------------------------------------
   OEM 单文件配置选项

   说明：在下方粘贴 cpu_map 和默认设置定义，不要使用
   #ifdef。注释掉本文件顶部的 CPU_MAP_xxx 和 DEFAULT_xxx 定义，编译器将忽略 defaults.h 和 cpu_map.h 的内容，
   并使用下面的定义。
*/

// 在此处粘贴 CPU_MAP 定义。

// 在此处粘贴 defaults.h 定义。

#endif
