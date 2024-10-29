/*
  gcode.h - rs274/ngc 解析器。
  属于 Grbl 项目的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon, Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以按照 GNU 通用公共许可证的条款进行重新分发和/或修改，
  该许可证由自由软件基金会发布，版本 3 或更高版本。

  Grbl 的发布目的是希望它能有用，
  但不提供任何担保，也没有任何隐含的
  适销性或适用于特定用途的保证。有关详细信息，请参阅
  GNU 通用公共许可证。

  您应该已收到随 Grbl 提供的 GNU 通用公共许可证的副本。
  如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#ifndef gcode_h
#define gcode_h

// 定义模态组内部编号，以便检查多重命令冲突和跟踪
// 在块中调用的命令类型。模态组是一个 G 代码命令组，
// 它们是互斥的，不能在同一行上存在，因为它们会切换状态或执行唯一的动作。
// 这些定义在 NIST RS274-NGC v3 G 代码标准中，可在线获取，
// 并且类似于其他制造商（Haas、Fanuc、Mazak 等）的 G 代码解析器。
// 注意：模态组的定义值必须从零开始并按顺序排列。
#define MODAL_GROUP_G0 0 // [G4, G10, G28, G28.1, G30, G30.1, G53, G92, G92.1] 非模态
#define MODAL_GROUP_G1 1 // [G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80] 运动
#define MODAL_GROUP_G2 2 // [G17, G18, G19] 平面选择
#define MODAL_GROUP_G3 3 // [G90, G91] 距离模式
#define MODAL_GROUP_G4 4 // [G91.1] 弧 IJK 距离模式
#define MODAL_GROUP_G5 5 // [G93, G94] 进给速率模式
#define MODAL_GROUP_G6 6 // [G20, G21] 单位
#define MODAL_GROUP_G7 7 // [G40] 刀具半径补偿模式。G41/42 不支持。
#define MODAL_GROUP_G8 8 // [G43.1, G49] 刀具长度补偿
#define MODAL_GROUP_G12 9 // [G54, G55, G56, G57, G58, G59] 坐标系选择
#define MODAL_GROUP_G13 10 // [G61] 控制模式

#define MODAL_GROUP_M4 11  // [M0, M1, M2, M30] 停止
#define MODAL_GROUP_M7 12 // [M3, M4, M5] 主轴旋转
#define MODAL_GROUP_M8 13 // [M7, M8, M9] 冷却控制

// #define OTHER_INPUT_F 14
// #define OTHER_INPUT_S 15
// #define OTHER_INPUT_T 16

// 为执行类型的模态组（运动、停止、非模态）定义命令动作。
// 内部使用在解析器中，用于确定要执行的命令。
// 注意：某些宏值被指定为特定值以使 G 代码状态报告和解析编译更小。
// 这在 328p 上完全耗尽闪存的情况下是必要的。尽管不理想，
// 但如果需要更改它们，仔细检查 report.c 和 gcode.c 中这些值的用法。

// 模态组 G0：非模态动作
#define NON_MODAL_NO_ACTION 0 // （默认：必须为零）
#define NON_MODAL_DWELL 4 // G4（不可更改值）
#define NON_MODAL_SET_COORDINATE_DATA 10 // G10（不可更改值）
#define NON_MODAL_GO_HOME_0 28 // G28（不可更改值）
#define NON_MODAL_SET_HOME_0 38 // G28.1（不可更改值）
#define NON_MODAL_GO_HOME_1 30 // G30（不可更改值）
#define NON_MODAL_SET_HOME_1 40 // G30.1（不可更改值）
#define NON_MODAL_ABSOLUTE_OVERRIDE 53 // G53（不可更改值）
#define NON_MODAL_SET_COORDINATE_OFFSET 92 // G92（不可更改值）
#define NON_MODAL_RESET_COORDINATE_OFFSET 102 //G92.1（不可更改值）

// 模态组 G1：运动模式
#define MOTION_MODE_SEEK 0 // G0（默认：必须为零）
#define MOTION_MODE_LINEAR 1 // G1（不可更改值）
#define MOTION_MODE_CW_ARC 2  // G2（不可更改值）
#define MOTION_MODE_CCW_ARC 3  // G3（不可更改值）
#define MOTION_MODE_PROBE_TOWARD 140 // G38.2（不可更改值）
#define MOTION_MODE_PROBE_TOWARD_NO_ERROR 141 // G38.3（不可更改值）
#define MOTION_MODE_PROBE_AWAY 142 // G38.4（不可更改值）
#define MOTION_MODE_PROBE_AWAY_NO_ERROR 143 // G38.5（不可更改值）
#define MOTION_MODE_NONE 80 // G80（不可更改值）

// 模态组 G2：平面选择
#define PLANE_SELECT_XY 0 // G17（默认：必须为零）
#define PLANE_SELECT_ZX 1 // G18（不可更改值）
#define PLANE_SELECT_YZ 2 // G19（不可更改值）

// 模态组 G3：距离模式
#define DISTANCE_MODE_ABSOLUTE 0 // G90（默认：必须为零）
#define DISTANCE_MODE_INCREMENTAL 1 // G91（不可更改值）

// 模态组 G4：弧 IJK 距离模式
#define DISTANCE_ARC_MODE_INCREMENTAL 0 // G91.1（默认：必须为零）

// 模态组 M4：程序流程
#define PROGRAM_FLOW_RUNNING 0 // （默认：必须为零）
#define PROGRAM_FLOW_PAUSED 3 // M0
#define PROGRAM_FLOW_OPTIONAL_STOP 1 // M1 注意：不支持，但有效并被忽略。
#define PROGRAM_FLOW_COMPLETED_M2  2 // M2（不可更改值）
#define PROGRAM_FLOW_COMPLETED_M30 30 // M30（不可更改值）

// 模态组 G5：进给速率模式
#define FEED_RATE_MODE_UNITS_PER_MIN  0 // G94（默认：必须为零）
#define FEED_RATE_MODE_INVERSE_TIME   1 // G93（不可更改值）

// 模态组 G6：单位模式
#define UNITS_MODE_MM 0 // G21（默认：必须为零）
#define UNITS_MODE_INCHES 1 // G20（不可更改值）

// 模态组 G7：刀具半径补偿模式
#define CUTTER_COMP_DISABLE 0 // G40（默认：必须为零）

// 模态组 G13：控制模式
#define CONTROL_MODE_EXACT_PATH 0 // G61（默认：必须为零）

// 模态组 M7：主轴控制
#define SPINDLE_DISABLE 0 // M5（默认：必须为零）
#define SPINDLE_ENABLE_CW   PL_COND_FLAG_SPINDLE_CW // M3（注意：使用规划器条件位标志）
#define SPINDLE_ENABLE_CCW  PL_COND_FLAG_SPINDLE_CCW // M4（注意：使用规划器条件位标志）

// 模态组 M8：冷却控制
#define COOLANT_DISABLE 0 // M9（默认：必须为零）
#define COOLANT_FLOOD_ENABLE  PL_COND_FLAG_COOLANT_FLOOD // M8（注意：使用规划器条件位标志）
#define COOLANT_MIST_ENABLE   PL_COND_FLAG_COOLANT_MIST  // M7（注意：使用规划器条件位标志）

// 模态组 G8：刀具长度补偿
#define TOOL_LENGTH_OFFSET_CANCEL 0 // G49（默认：必须为零）
#define TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC 1 // G43.1

// 模态组 G12：活动工作坐标系
// 不适用：存储坐标系值（54-59）以进行更改。

// 定义参数字映射。
#define WORD_F  0
#define WORD_I  1
#define WORD_J  2
#define WORD_K  3
#define WORD_L  4
#define WORD_N  5
#define WORD_P  6
#define WORD_R  7
#define WORD_S  8
#define WORD_T  9
#define WORD_X  10
#define WORD_Y  11
#define WORD_Z  12

#define WORD_A  13
#define WORD_B  14
#define WORD_C  15

// 定义 G 代码解析器位置更新标志
#define GC_UPDATE_POS_TARGET   0 // 必须为 0
#define GC_UPDATE_POS_SYSTEM   1
#define GC_UPDATE_POS_NONE     2

// 定义探针循环退出状态并分配适当的位置更新
#define GC_PROBE_FOUND      GC_UPDATE_POS_SYSTEM
#define GC_PROBE_ABORT      GC_UPDATE_POS_NONE
#define GC_PROBE_FAIL_INIT  GC_UPDATE_POS_NONE
#define GC_PROBE_FAIL_END   GC_UPDATE_POS_TARGET
#ifdef SET_CHECK_MODE_PROBE_TO_START
  #define GC_PROBE_CHECK_MODE   GC_UPDATE_POS_NONE  
#else
  #define GC_PROBE_CHECK_MODE   GC_UPDATE_POS_TARGET
#endif

// 定义用于处理特殊情况的 G 代码解析器标志
#define GC_PARSER_NONE                  0 // 必须为 0
#define GC_PARSER_JOG_MOTION            bit(0)
#define GC_PARSER_CHECK_MANTISSA        bit(1)
#define GC_PARSER_ARC_IS_CLOCKWISE      bit(2)
#define GC_PARSER_PROBE_IS_AWAY         bit(3)
#define GC_PARSER_PROBE_IS_NO_ERROR     bit(4)
#define GC_PARSER_LASER_FORCE_SYNC      bit(5)
#define GC_PARSER_LASER_DISABLE         bit(6)
#define GC_PARSER_LASER_ISMOTION        bit(7)


// 注意：当该结构体清零时，上述定义设置系统默认值。
typedef struct {
  uint8_t motion;          // {G0,G1,G2,G3,G38.2,G80}
  uint8_t feed_rate;       // {G93,G94}
  uint8_t units;           // {G20,G21}
  uint8_t distance;        // {G90,G91}
  // uint8_t distance_arc; // {G91.1} 注意：不跟踪，仅支持默认值
  uint8_t plane_select;    // {G17,G18,G19}
  // uint8_t cutter_comp;  // {G40} 注意：不跟踪，仅支持默认值
  uint8_t tool_length;     // {G43.1,G49}
  uint8_t coord_select;    // {G54,G55,G56,G57,G58,G59}
  // uint8_t control;      // {G61} 注意：不跟踪，仅支持默认值
  uint8_t program_flow;    // {M0,M1,M2,M30}
  uint8_t coolant;         // {M7,M8,M9}
  uint8_t spindle;         // {M3,M4,M5}
} gc_modal_t;

typedef struct {
  float f;         // 进给速度
  float ijk[N_AXIS];    // I、J、K 轴弧偏移
  uint8_t l;       // G10 或循环参数
  int32_t n;       // 行号
  float p;         // G10 或延时参数
  // float q;      // G82 挖槽钻孔
  float r;         // 弧半径
  float s;         // 主轴转速
  uint8_t t;       // 工具选择
  float xyz[N_AXIS];    // X、Y、Z 平移轴
} gc_values_t;


typedef struct {
  gc_modal_t modal;

  float spindle_speed;          // 转速 RPM
  float feed_rate;              // 毫米/分钟
  uint8_t tool;                 // 跟踪工具编号，未使用
  int32_t line_number;          // 最后发送的行号

  float position[N_AXIS];       // 解释器认为代码中的工具当前所在的位置

  float coord_system[N_AXIS];    // 当前工作坐标系 (G54+)。存储相对于绝对机床位置的偏移量（以 mm 为单位），在调用时从 EEPROM 加载。
  float coord_offset[N_AXIS];    // 保留 G92 坐标偏移（工作坐标）相对于机床零点的偏移量（以 mm 为单位），非持久性。在复位和启动时清除。
  float tool_length_offset;      // 跟踪启用时的工具长度偏移值。
} parser_state_t;
extern parser_state_t gc_state;


typedef struct {
  uint8_t non_modal_command;
  gc_modal_t modal;
  gc_values_t values;
} parser_block_t;


// 初始化解析器
void gc_init();

// 执行一行 rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

// 设置 G 代码解析器位置，输入为步进。
void gc_sync_position();

#endif
