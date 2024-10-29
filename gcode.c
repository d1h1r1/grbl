/*
  gcode.c - rs274/ngc 解析器。
  属于 Grbl 项目的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是免费软件：您可以在 GNU 通用公共许可证的条款下重新分发和/或修改，
  该许可证由自由软件基金会发布，许可证版本为 3 或更高版本。

  Grbl 分发时希望它有用，
  但不提供任何保证；甚至不保证适销性或适合特定用途。
  有关详细信息，请参阅 GNU 通用公共许可证。

  您应该已经收到了 Grbl 附带的 GNU 通用公共许可证副本。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 注意：G 代码标准将最大行号定义为 99999。该值似乎是任意的，
// 一些 GUI 可能需要更大的值。因此，我们基于将浮点数
// 转换为整数的最大安全值（7.2 位精度）增加了该值。
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // 受限于最大无符号 8 位值

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *未定义但必需

// 声明 gc 外部结构
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);


// 初始化 g-code 解析器
void gc_init()
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // 加载默认的 G54 坐标系。
  if (!(settings_read_coord_data(gc_state.modal.coord_select, gc_state.coord_system))) {
    report_status_message(STATUS_SETTING_READ_FAIL);
  }
}


// 设置 g-code 解析器位置（单位：毫米）。输入单位为步。由系统中止和硬限位
// 偏移例程调用。
void gc_sync_position()
{
  system_convert_array_steps_to_mpos(gc_state.position, sys_position);
}


// 执行一行以 0 结尾的 G 代码。假设该行只包含大写字符和带符号的浮点值
// （无空格）。注释和块删除字符已被删除。在此函数中，所有单位和位置
// 都被转换并以（毫米，毫米/分钟）和绝对机床坐标的形式导出到 grbl 的内部功能。
uint8_t gc_execute_line(char *line)
{
  /* -------------------------------------------------------------------------------------
     第 1 步：初始化解析器块结构体并复制当前 g-code 状态模式。解析器
     会在解析块行时更新这些模式和命令，并且只有在成功的错误检查后才会使用和执行。
     解析器块结构体还包含一个块值结构体、字跟踪变量和一个新的非模态命令跟踪器。
     该结构体包含执行该块所需的所有信息。
  */


  memset(&gc_block, 0, sizeof(parser_block_t)); // 初始化解析器块结构体。
memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_modal_t)); // 复制当前模式。

uint8_t axis_command = AXIS_COMMAND_NONE;
uint8_t axis_0, axis_1, axis_linear;
uint8_t coord_select = 0; // 跟踪执行的 G10 P 坐标选择。

// 初始化用于跟踪与轴索引兼容操作的标志位变量。
uint8_t axis_words = 0; // XYZ 跟踪。
uint8_t ijk_words = 0; // IJK 跟踪。

// 初始化命令和数值字以及解析器标志变量。
uint16_t command_words = 0; // 跟踪 G 和 M 命令字。还用于检测模式组冲突。
uint16_t value_words = 0; // 跟踪数值字。
uint8_t gc_parser_flags = GC_PARSER_NONE;

// 确定该行是快速移动还是普通的 g-code 块。
if (line[0] == '$') { // 注意：`$J=` 在传递到此函数时已解析。
  // 设置 G1 和 G94 强制模式以确保准确的错误检查。
  gc_parser_flags |= GC_PARSER_JOG_MOTION;
  gc_block.modal.motion = MOTION_MODE_LINEAR;
  gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
  gc_block.values.n = JOG_LINE_NUMBER; // 初始化在快速移动期间报告的默认行号。
}

/* -------------------------------------------------------------------------------------
   第 2 步：导入块行中的所有 g-code 字。一个 g-code 字是一个字母加一个数字，
   它可以是 'G'/'M' 命令，也可以是设置/分配命令值。同时执行初步错误检查，
   以检测命令字模式组冲突、任何重复字以及 F、N、P、T 和 S 的数值字的负值。 */

uint8_t word_bit; // 用于分配跟踪变量的位值
uint8_t char_counter;
char letter;
float value;
uint8_t int_value = 0;
uint16_t mantissa = 0;
if (gc_parser_flags & GC_PARSER_JOG_MOTION) { char_counter = 3; } // 从 `$J=` 之后开始解析
else { char_counter = 0; }

while (line[char_counter] != 0) { // 循环直到行中没有更多的 g-code 字。

  // 导入下一个 g-code 字，期望是字母后跟数值，否则返回错误。
  letter = line[char_counter];
  if ((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [预期字母]
  char_counter++;
  if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [预期数值]

  // 将数值转换为较小的 uint8 有效数和尾数值，以解析该字。
  // 注意：尾数乘以 100 以捕捉非整数命令值。比 NIST gcode 对命令要求的 x10 更精确，
  // 但对要求精确到 0.0001 的数值字不够精确。这是一个良好的折中方案，能捕捉到大多数非整数错误。
  // 为完全符合标准，只需将尾数改为 int16，但这会增加编译后所占的闪存空间。未来可能会更新。
  int_value = trunc(value);
  mantissa = round(100 * (value - int_value)); // 计算 Gxx.x 命令的尾数。
  // 注意：必须使用四舍五入以捕捉小的浮点错误。

  // 检查 g-code 字是否受支持、是否因模式组冲突错误或在 g-code 块中被重复。如果有效，则更新命令或记录其值。
  switch (letter) {

    /* 'G' 和 'M' 命令字：解析命令并检查模式组冲突。
       注意：模式组编号定义在 NIST RS274-NGC v3, 第 20 页的表 4 中。 */

    case 'G':
      // 确定 'G' 命令及其模式组。
      switch (int_value) {
        case 10: case 28: case 30: case 92:
          // 检查在同一块中调用 G0/1/2/3/38 时是否调用了 G10/28/30/92。
          // * G43.1 也是一个轴命令，但未显式定义为此。
          if (mantissa == 0) { // 忽略 G28.1、G30.1 和 G92.1。
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [轴字/命令冲突]
            axis_command = AXIS_COMMAND_NON_MODAL;
          }
          // 没有中断，继续下一行。
        case 4: case 53:
          word_bit = MODAL_GROUP_G0;
          gc_block.non_modal_command = int_value;
          if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
            if (!((mantissa == 0) || (mantissa == 10))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); }
            gc_block.non_modal_command += mantissa;
            mantissa = 0; // 设置为零以指示有效的非整数 G 命令。
          }                
          break;
        case 0: case 1: case 2: case 3: case 38:
          // 检查在同一块中调用 G10/28/30/92 时是否调用了 G0/1/2/3/38。
          // * G43.1 也是一个轴命令，但未显式定义为此。
          if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [轴字/命令冲突]
          axis_command = AXIS_COMMAND_MOTION_MODE;
          // 没有中断，继续下一行。
        case 80:
          word_bit = MODAL_GROUP_G1;
          gc_block.modal.motion = int_value;
          if (int_value == 38){
            if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))) {
              FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [不支持的 G38.x 命令]
            }
            gc_block.modal.motion += (mantissa/10) + 100;
            mantissa = 0; // 设置为零以指示有效的非整数 G 命令。
          }  
          break;
        case 17: case 18: case 19:
          word_bit = MODAL_GROUP_G2;
          gc_block.modal.plane_select = int_value - 17;
          break;
        case 90: case 91:
          if (mantissa == 0) {
            word_bit = MODAL_GROUP_G3;
            gc_block.modal.distance = int_value - 90;
          } else {
            word_bit = MODAL_GROUP_G4;
            if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [不支持 G90.1]
            mantissa = 0; // 设置为零以指示有效的非整数 G 命令。
            // 否则，IJK 增量模式默认。G91.1 无操作。
          }
          break;
        case 93: case 94:
          word_bit = MODAL_GROUP_G5;
          gc_block.modal.feed_rate = 94 - int_value;
          break;
        case 20: case 21:
          word_bit = MODAL_GROUP_G6;
          gc_block.modal.units = 21 - int_value;
          break;
        case 40:
          word_bit = MODAL_GROUP_G7;
          // 注意：不需要，因为刀具半径补偿总是禁用的。仅为支持 G40 命令以设定程序的默认设置。
          // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
          break;
        case 43: case 49:
          word_bit = MODAL_GROUP_G8;
          // 注意：NIST g-code 标准模糊地指出，当更改刀具长度补偿时，
          // 不能更新任何轴运动或坐标偏移。即 G43、G43.1 和 G49 均为显式轴命令，
          // 无论它们是否需要轴字。
          if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [轴字/命令冲突] }
          axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
          if (int_value == 49) { // G49
            gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
          } else if (mantissa == 10) { // G43.1
            gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
          } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [不支持 G43.x 命令]
          mantissa = 0; // 设置为零以指示有效的非整数 G 命令。
          break;
        case 54: case 55: case 56: case 57: case 58: case 59:
          // 注意：不支持 G59.x（但其 int_value 分别为 60、61 和 62）。
          word_bit = MODAL_GROUP_G12;
          gc_block.modal.coord_select = int_value - 54; // 转换为数组索引。
          break;
        case 61:
          word_bit = MODAL_GROUP_G13;
          if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [不支持 G61.1]
          // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
          break;
        default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [不支持的 G 命令]
      }
      if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [不支持或无效的 Gxx.x 命令]
      // 检查当前块中是否存在每个模式组多个命令的冲突
      // 注意：如果命令有效，变量 'word_bit' 始终被赋值。
      if (bit_istrue(command_words, bit(word_bit))) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
      command_words |= bit(word_bit);
      break;


      case 'M':

        // 确定 'M' 命令及其模式组
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [不允许 Mxx.x 命令]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // 暂停程序
              case 1: break; // 不支持可选停止。忽略。
              default: gc_block.modal.program_flow = int_value; // 程序结束并复位
            }
            break;
          case 3: case 4: case 5:
            word_bit = MODAL_GROUP_M7;
            switch(int_value) {
              case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;
              case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;
              case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;
            }
            break;            
          case 7: case 8: case 9:
            word_bit = MODAL_GROUP_M8; 
            switch(int_value) {      
              case 7: gc_block.modal.coolant = COOLANT_MIST_ENABLE; break;
              case 8: gc_block.modal.coolant = COOLANT_FLOOD_ENABLE; break;
              case 9: gc_block.modal.coolant = COOLANT_DISABLE; break;
            }
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [不支持的 M 命令]
        }

        // 检查当前块中每个模式组是否包含超过一个命令
        // 注意：如果命令有效，则始终会分配变量 'word_bit'。
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      // 注意：所有剩余字母都分配值。
      default:

        /* 非命令字：此初始解析阶段仅检查其余合法的 G 代码字是否重复，并存储其值。
           错误检查将在稍后进行，因为某些字母（I、J、K、L、P、R）具有多重含义，且/或取决于发出的命令。 */
        switch(letter){
#ifdef A_AXIS
          case 'A': word_bit = WORD_A; gc_block.values.xyz[A_AXIS] = value; axis_words |= (1<<A_AXIS); break;
#endif
#ifdef B_AXIS
          case 'B': word_bit = WORD_B; gc_block.values.xyz[B_AXIS] = value; axis_words |= (1<<B_AXIS); break;
#endif
#ifdef C_AXIS
          case 'C': word_bit = WORD_C; gc_block.values.xyz[C_AXIS] = value; axis_words |= (1<<C_AXIS); break;
#endif

          // case 'A': // 不支持
          // case 'B': // 不支持
          // case 'C': // 不支持
          // case 'D': // 不支持
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          // case 'H': // 不支持
          case 'I': word_bit = WORD_I; gc_block.values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          // 注意：对于某些命令，P 值必须为整数，但这些命令均不支持。
          // case 'Q': // 不支持
          case 'R': word_bit = WORD_R; gc_block.values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block.values.s = value; break;
          case 'T': word_bit = WORD_T; 
					  if (value > MAX_TOOL_NUMBER) { FAIL(STATUS_GCODE_MAX_VALUE_EXCEEDED); }
            gc_block.values.t = int_value;
						break;
          case 'X': word_bit = WORD_X; gc_block.values.xyz[X_AXIS] = value; axis_words |= (1<<X_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block.values.xyz[Y_AXIS] = value; axis_words |= (1<<Y_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block.values.xyz[Z_AXIS] = value; axis_words |= (1<<Z_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }

        // 注意：如果非命令字母有效，则始终会分配变量 'word_bit'。
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } // [字母重复]
        // 检查字母 F、N、P、T 和 S 的负值无效
        // 注意：为提高代码效率，负值检查在此进行。
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } // [字母值不能为负]
        }
        value_words |= bit(word_bit); // 标记分配的参数。

    }
  }
  // 解析完成！


  /* -------------------------------------------------------------------------------------
     步骤 3：检查此块中传递的所有命令和值中的错误。此步骤确保所有命令均可执行，并尽可能接近 NIST 标准。
     如果发现错误，将丢弃此块中的所有命令和值，不会更新活动系统的 G 代码模式。
     如果块正确，则会根据此块中的命令更新活动系统的 G 代码模式，并发出执行信号。

     此外，我们必须基于已解析块设置的模式预转换所有传递的值。由于某些错误检查需要准确计算目标信息，
     只能在结合错误检查时转换这些值。
     因此，将仅需更新系统的 G 代码模式并按顺序执行编程操作，执行步骤不应涉及任何转换计算，且只需进行最少的检查以执行操作。
  */

  /* 注意：在此时，G 代码块已解析，块行可以释放。
     注意：未来的某个时刻，可以分解步骤 2，以允许按字母逐字解析块，而不是整个块。
     这可以消除需要维护整个块的大字符串变量，并释放一些内存。
     要做到这一点，只需保留步骤 1 中的所有数据，如新块数据结构、模式组和值位标记变量以及轴数组索引兼容变量。
     此数据包含所有必要信息，以在收到 EOL 字符时检查新 G 代码块的错误。
     然而，这会打破 Grbl 的启动行的当前工作方式，并需要一些重构以使其兼容。
  */

  // [0. 非特定/常见错误检查及其他设置]：

  // 确定隐式轴命令条件。传递了轴字，但未发送显式轴命令。如果是，则将轴命令设置为当前运动模式。
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } // 分配隐式运动模式
  }

  // 检查有效的行号 N 值。
  if (bit_istrue(value_words,bit(WORD_N))) {
    // 行号值不能小于零（已完成）或大于最大行号。
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [超出最大行号]
  }
  // bit_false(value_words,bit(WORD_N)); // 注意：单含义值字母。在错误检查结束时设置。

  // 在错误检查结束时跟踪未使用的字母。
  // 注意：单含义值字母一次性删除，因为它们总是在存在时使用。这是为了节省几字节的闪存。为了清晰，可以在使用它们时删除。轴字母同样对待。
  // 如果存在显式/隐式轴命令，XYZ 字母总会被使用，并在错误检查结束时被移除。

  // [1. 注释]：不支持 MSG。注释处理由协议执行。

  // [2. 设置进给率模式]：G93 F 字母缺失且 G1、G2/3 活动，隐式或显式地。切换到 G94 后，进给率未定义。
  // 注意：对于点动，忽略先前的进给率模式。强制使用毫秒模式

  // 若开启了手动移动模式 (JOG)，则需要检查进给率是否已定义
if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    // 若 F 字未定义则返回错误状态
    if (bit_isfalse(value_words, bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); }
    // 若单位为英寸，则将 F 值转换为毫米
    if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
} else {
    // 检查进给率模式，若为逆时间模式 (G93)
    if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // G93
        // 注意：G38 也可以在逆时间模式下工作，但若未定义将返回错误
        if (axis_command == AXIS_COMMAND_MOTION_MODE) {
            if ((gc_block.modal.motion != MOTION_MODE_NONE) && (gc_block.modal.motion != MOTION_MODE_SEEK)) {
                // 若未定义 F 字，则返回进给率未定义错误
                if (bit_isfalse(value_words, bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); }
            }
        }
        // NOTE: 若从 G94 切换到 G93，检查 F 字是否传递。可直接重置进给率值，未定义则设置为零
        // 可使 G93 模式下，缺少 F 字时错误抛出，但若不需要进给率时，则不报错。
    } else { // G94 单位模式
        // 若为 mm/min 模式下：若 F 字传递，确保值为 mm/min，否则使用上次状态值
        if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { // 上次状态也是 G94
            if (bit_istrue(value_words, bit(WORD_F))) {
                if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
            } else {
                gc_block.values.f = gc_state.feed_rate; // 推送上次状态的进给率
            }
        } // 否则从 G93 切换至 G94，进给率未定义或传递的 F 字值生效
    }
}
// bit_false(value_words, bit(WORD_F)); // 单一意义值字的设置。错误检查结束时设置

// [设置主轴速度] 若 S 字未定义则使用上次状态值
if (bit_isfalse(value_words, bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }
// bit_false(value_words, bit(WORD_S)); // 单一意义值字的设置。错误检查结束时设置

// [选择刀具]：仅跟踪值。若 T 字非整数或大于最大刀具值则返回错误
// bit_false(value_words, bit(WORD_T)); // 单一意义值字的设置。错误检查结束时设置

// [暂停]：P 值缺失或为负时返回错误
if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words, bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // P 字缺失
    bit_false(value_words, bit(WORD_P));
}

// [设定活动平面]
switch (gc_block.modal.plane_select) {
    case PLANE_SELECT_XY:
        axis_0 = X_AXIS;
        axis_1 = Y_AXIS;
        axis_linear = Z_AXIS;
        break;
    case PLANE_SELECT_ZX:
        axis_0 = Z_AXIS;
        axis_1 = X_AXIS;
        axis_linear = Y_AXIS;
        break;
    default: // PLANE_SELECT_YZ
        axis_0 = Y_AXIS;
        axis_1 = Z_AXIS;
        axis_linear = X_AXIS;
}

// [长度单位设定]：如适用，将 XYZ 坐标值预转换为毫米
uint8_t idx;
if (gc_block.modal.units == UNITS_MODE_INCHES) {
    for (idx = 0; idx < N_AXIS; idx++) { // 轴索引一致，可使用循环
        if (bit_istrue(axis_words, bit(idx))) {
            gc_block.values.xyz[idx] *= MM_PER_INCH;
        }
    }
}

// [刀具半径补偿]：G41/42 不支持。若 G53 激活则报错
// [G40 错误]：G40 后执行 G2/3 圆弧或在禁用后移动距离小于刀具直径
// 注意：因未启用刀具半径补偿，不适用 G40 错误。G40 的作用仅为避免程序头设定模式时出错

// [刀具长度补偿]：G43 不支持，但 G43.1 和 G49 支持。
// [G43.1 错误]：同行出现运动指令
if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET) { // 表示在代码块中调用
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) {
        if (axis_words ^ (1 << TOOL_LENGTH_OFFSET_AXIS)) { FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR); }
    }
}

// [坐标系统选择]：如刀具半径补偿激活则报错
// TODO：EEPROM 读取坐标数据时或需同步缓冲区以防活跃周期中造成的崩溃
float block_coord_system[N_AXIS];
memcpy(block_coord_system, gc_state.coord_system, sizeof(gc_state.coord_system));
if (bit_istrue(command_words, bit(MODAL_GROUP_G12))) { // 检查是否在代码块中调用
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // 大于 N
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
        if (!(settings_read_coord_data(gc_block.modal.coord_select, block_coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
    }
}

// [路径控制模式设定]：仅支持 G61，不支持 G61.1 和 G64
// [距离模式设定]：仅支持 G91.1，不支持 G90.1
// [缩回模式设定]：不支持

// [剩余非模态操作]：检查是否进入预设位置、设定 G10 或轴偏移
switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
        // [G10 错误]：缺少 L 且非 2 或 20。缺少 P 字
        // [G10 L2 错误]：R 字不支持，P 值超出范围，缺少轴字
        // [G10 L20 错误]：P 值需为 0 至 nCoordSys(最大为 9)，缺少轴字
        if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // 无轴字
        if (bit_isfalse(value_words, ((1 << WORD_P) | (1 << WORD_L)))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // P/L 字缺失
        coord_select = trunc(gc_block.values.p); // 将 p 值转换为整数
        if (coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // 大于 N
        if (gc_block.values.l != 20) {
            if (gc_block.values.l == 2) {
                if (bit_istrue(value_words, bit(WORD_R))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // 不支持 G10 L2 R
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // 不支持的 L
        }
        bit_false(value_words, (bit(WORD_L) | bit(WORD_P)));

        // 确定要更改的坐标系统并尝试从 EEPROM 加载
        if (coord_select > 0) { coord_select--; } // 将 P1-P6 索引调整至 EEPROM 索引
        else { coord_select = gc_block.modal.coord_select; } // P0 索引为活动坐标系

        // 注意：参数数据存储于 IJK 值。规则规定，该指令不使用 IJK
        // FIXME: 建议使用 float vector[N_AXIS] 代替 IJK
        if (!settings_read_coord_data(coord_select, gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); } // EEPROM 读取失败

        // 预计算坐标数据的更改
        for (idx = 0; idx < N_AXIS; idx++) { // 轴索引一致，可使用循环
            // 仅更新块中定义的轴，始终为机器坐标，可更改非活动系统
            if (bit_istrue(axis_words, bit(idx))) {
                if (gc_block.values.l == 20) {
                    // L20: 按当前位置 (包含偏移) 更新坐标系统轴
                    gc_block.values.ijk[idx] = gc_state.position[idx] - gc_state.coord_offset[idx] - gc_block.values.xyz[idx];
                    if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.ijk[idx] -= gc_state.tool_length_offset; }
                } else {
                    // L2: 将坐标系统轴更新至设定值
                    gc_block.values.ijk[idx] = gc_block.values.xyz[idx];
                }
            }
        }
        break;

    case NON_MODAL_SET_COORDINATE_OFFSET:
      // [G92 错误]：没有指定轴字。
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [没有轴字]

      // 仅更新代码块中定义的轴。将当前坐标系统偏移到定义的值。不会在选择当前坐标系统时更新，但仍处于激活状态，除非 G92.1 禁用它。
      for (idx=0; idx<N_AXIS; idx++) { // 轴索引是一致的，因此可以使用循环。
        if (bit_istrue(axis_words,bit(idx)) ) {
          // WPos = MPos - WCS - G92 - TLO  ->  G92 = MPos - WCS - TLO - WPos
          gc_block.values.xyz[idx] = gc_state.position[idx]-block_coord_system[idx]-gc_block.values.xyz[idx];
          if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] -= gc_state.tool_length_offset; }
        } else {
          gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
        }
      }
      break;
      
    default:

      // 此时，其他显式轴指令将轴值视为传统的目标位置，应用了坐标系统偏移、G92 偏移、绝对覆盖和距离模式。这包括运动模式命令。我们现在可以预先计算目标位置。
      // 注意：工具偏移可能会在添加此功能时附加到这些转换中。
      if (axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // TLO 阻止任何轴命令。
        if (axis_words) {
          for (idx=0; idx<N_AXIS; idx++) { // 轴索引是一致的，因此可以使用循环以节省内存空间。
            if ( bit_isfalse(axis_words,bit(idx)) ) {
              gc_block.values.xyz[idx] = gc_state.position[idx]; // 代码块中没有轴字。保持同一轴位置。
            } else {
              // 根据距离模式更新指定值，或者在启用绝对覆盖时忽略。
              // 注意：G53 在 G28/30 下从不激活，因为它们属于相同的模态组。
              if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
                // 根据距离模式应用坐标偏移。
                if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {
                  gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
                  if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] += gc_state.tool_length_offset; }
                } else {  // 增量模式
                  gc_block.values.xyz[idx] += gc_state.position[idx];
                }
              }
            }
          }
        }
      }

      // 检查其余非模态命令是否存在错误。
      switch (gc_block.non_modal_command) {
        case NON_MODAL_GO_HOME_0: // G28
        case NON_MODAL_GO_HOME_1: // G30
          // [G28/30 错误]：刀具补偿已启用。
          // 从 EEPROM 中检索 G28/30 返回原点位置数据（以机器坐标表示）
          // 注意：将参数数据存储在 IJK 值中。按规定，它们在此命令中未使用。
          if (gc_block.non_modal_command == NON_MODAL_GO_HOME_0) {
            if (!settings_read_coord_data(SETTING_INDEX_G28,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          } else { // == NON_MODAL_GO_HOME_1
            if (!settings_read_coord_data(SETTING_INDEX_G30,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          }
          if (axis_words) {
            // 仅移动在次级移动中指定的轴。
            for (idx=0; idx<N_AXIS; idx++) {
              if (!(axis_words & (1<<idx))) { gc_block.values.ijk[idx] = gc_state.position[idx]; }
            }
          } else {
            axis_command = AXIS_COMMAND_NONE; // 如果没有中间移动，则设置为无。
          }
          break;
        case NON_MODAL_SET_HOME_0: // G28.1
        case NON_MODAL_SET_HOME_1: // G30.1
          // [G28.1/30.1 错误]：刀具补偿已启用。
          // 注意：如果此处传入轴字，则将其解释为隐式运动模式。
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET:
          // 注意：如果此处传入轴字，则将其解释为隐式运动模式。
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
          // [G53 错误]：G0 和 G1 未激活。刀具补偿已启用。
          // 注意：所有显式轴字命令均在此模态组中。因此不需要隐式检查。
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 未激活]
          }
          break;
      }
  }

  // [20. 运动模式]：
if (gc_block.modal.motion == MOTION_MODE_NONE) {
  // [G80 错误]：在 G80 激活时，编程了轴字。
  // 注意：即使是使用轴字的非模态命令或 TLO 也会引发此严格错误。
  if (axis_words) { FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); } // [不允许轴字]

  // 检查剩余的运动模式，如果轴字是隐式的（存在且未被 G10/28/30/92 使用）或在 g 代码块中显式命令。
} else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {

  if (gc_block.modal.motion == MOTION_MODE_SEEK) {
    // [G0 错误]：轴字母未配置或没有真实值（已完成）。
    // 轴字是可选的。如果缺失，则将轴命令标志设置为忽略执行。
    if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

  // 所有剩余的运动模式（除 G0 和 G80 外），都需要一个有效的进给率值。在单位为 mm 模式中，
  // 该值必须为正数。在反时间模式中，必须在每个块中传递一个正值。
  } else {
    // 检查进给率是否已定义，适用于需要进给率的运动模式。
    if (gc_block.values.f == 0.0) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [进给率未定义]

    switch (gc_block.modal.motion) {
      case MOTION_MODE_LINEAR:
        // [G1 错误]：进给率未定义。轴字母未配置或没有真实值。
        // 轴字是可选的。如果缺失，则将轴命令标志设置为忽略执行。
        if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }
        break;
      case MOTION_MODE_CW_ARC: 
        gc_parser_flags |= GC_PARSER_ARC_IS_CLOCKWISE; // 故意无 break。
      case MOTION_MODE_CCW_ARC:
        // [G2/3 错误 所有模式]：进给率未定义。
        // [G2/3 半径模式错误]：在选定平面中无轴字。目标点与当前相同。
        // [G2/3 偏移模式错误]：在选定平面中无轴字和/或偏移。当前点和目标点的半径相差超过 0.002mm（EMC 定义为 0.5mm 或 0.005mm 和 0.1% 半径）。
        // [G2/3 完全圆模式错误]：不支持。存在轴字。未编程偏移。P 必须为整数。
        // 注意：半径和偏移量都用于圆弧跟踪，并在错误检查时预先计算。

        if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [无轴字]
        if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); } // [平面中无轴字]

        // 计算沿每个选定轴的位移量
        float x,y;
        x = gc_block.values.xyz[axis_0]-gc_state.position[axis_0]; // 当前位置与目标之间的 x 位移
        y = gc_block.values.xyz[axis_1]-gc_state.position[axis_1]; // 当前位置与目标之间的 y 位移

        if (value_words & bit(WORD_R)) { // 圆弧半径模式
          bit_false(value_words,bit(WORD_R));
          if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [无效目标]

          // 将半径值转换为合适的单位。
          if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }
          /* 我们需要计算具有指定半径并通过当前位置和目标位置的圆心。此方法计算如下方程组，
            其中 [x,y] 是从当前位置到目标位置的向量，d == 该向量的大小，h == 由圆的半径形成的三角形的斜边，
            旅行向量的中心到圆心的距离。垂直于旅行向量的向量 [-y,x] 缩放为 h 长度 [-y/d*h, x/d*h]，
            并添加到旅行向量中心 [x/2,y/2]，形成新点 [i,j]，即我们圆弧的中心。
            d^2 == x^2 + y^2
            h^2 == r^2 - (d/2)^2
            i == x/2 - y/d*h
            j == y/2 + x/d*h

                                                                  O <- [i,j]
                                                              -  |
                                                    r      -     |
                                                        -        |
                                                      -           | h
                                                  -              |
                                    [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                              | <------ d/2 ---->|

            C - Current position
            T - Target position
            O - center of circle that pass through both C and T
            d - distance from C to T
            r - designated radius
            h - distance from center of CT to O

            Expanding the equations:

            d -> sqrt(x^2 + y^2)
            h -> sqrt(4 * r^2 - x^2 - y^2)/2
            i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
            j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

            Which can be written:

            i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
            j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

            Which we for size and speed reasons optimize to:

            h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
            i = (x - (y * h_x2_div_d))/2
            j = (y + (x * h_x2_div_d))/2
            */

          // 先用 h_x2_div_d 计算 4*h^2，检查是否为负或 r 小于 d。
          float h_x2_div_d = 4.0 * gc_block.values.r*gc_block.values.r - x*x - y*y;

          if (h_x2_div_d < 0) { FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); } // [圆弧半径错误]

          // 完成 h_x2_div_d 的计算。
          h_x2_div_d = -sqrt(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
          // 如果圆为逆时针方向，则反转 h_x2_div_d 的符号。
          if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

          /* // 逆时针圆位于目标方向的左侧。当偏移为正时，生成左侧圆；
               // 当偏移为负时，生成右侧圆。

                                                                   T  <-- Target position

                                                                   ^
                        Clockwise circles with this center         |          Clockwise circles with this center will have
                        will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                         \         |          /
            center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                   |
                                                                   |

                                                                   C  <-- Current position
            */
            // 负 R 在 G-code 中是指“我想要一个超过 180 度的圆”（想想看！），
            // 尽管建议永远不要在单行 G-code 中生成这样的圆。通过
            // 反转 h_x2_div_d 的符号，圆心被放置在运动线路的另一侧，
            // 因此我们得到了不建议的长弧。
          if (gc_block.values.r < 0) {
              h_x2_div_d = -h_x2_div_d;
              gc_block.values.r = -gc_block.values.r; // 完成 r 的使用，将其设置为正数供 mc_arc 使用。
          }
          // 最后计算圆弧的实际中心。
          gc_block.values.ijk[axis_0] = 0.5*(x-(y*h_x2_div_d));
          gc_block.values.ijk[axis_1] = 0.5*(y+(x*h_x2_div_d));

        } else { // 圆心偏移模式
          if (!(ijk_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE); } // [平面中无偏移]
          bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_K)));

          // 将 IJK 值转换为合适的单位。
          if (gc_block.modal.units == UNITS_MODE_INCHES) {
            for (idx=0; idx<N_AXIS; idx++) { // 轴索引一致，因此可使用循环节省空间。
              if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
            }
          }

          // 从圆心到目标的弧半径
          x -= gc_block.values.ijk[axis_0]; // 圆心与目标之间的 x 位移
          y -= gc_block.values.ijk[axis_1]; // 圆心与目标之间的 y 位移
          float target_r = hypot_f(x,y);

          // 计算 mc_arc 的弧半径。从当前位置到圆心的定义。
          gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);

          // 计算当前位置和目标半径之间的差异，以进行最终错误检查。
          float delta_r = fabs(target_r-gc_block.values.r);
          if (delta_r > 0.005) {
            if (delta_r > 0.5) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [弧定义错误] > 0.5mm
            if (delta_r > (0.001*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [弧定义错误] > 0.005mm 且 半径的 0.1%
          }
        }
        break;
      case MOTION_MODE_PROBE_TOWARD_NO_ERROR: case MOTION_MODE_PROBE_AWAY_NO_ERROR:
        gc_parser_flags |= GC_PARSER_PROBE_IS_NO_ERROR; // 故意无 break。
      case MOTION_MODE_PROBE_TOWARD: case MOTION_MODE_PROBE_AWAY:
        if ((gc_block.modal.motion == MOTION_MODE_PROBE_AWAY) || 
            (gc_block.modal.motion == MOTION_MODE_PROBE_AWAY_NO_ERROR)) { gc_parser_flags |= GC_PARSER_PROBE_IS_AWAY; }
        // [G38 错误]：目标与当前相同。无轴字。刀具补偿已启用。进给率未定义。探针已触发。
        // 注意：探针检查已移至探测周期。为了防止进一步的运动到探针，发出警报。
         if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          break;
      }
    }
  }


  // [21. 程序流程]: 不需要错误检查。

// [0. 非特定错误检查]: 完成未使用值字的检查，即在弧半径模式下使用 IJK，或在区块中未使用的轴字。
if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    // Jogging 仅使用 F 进给速率和 XYZ 值字。N 是有效的，但 S 和 T 是无效的。
    bit_false(value_words,(bit(WORD_N)|bit(WORD_F)));
} else {
    bit_false(value_words,(bit(WORD_N)|bit(WORD_F)|bit(WORD_S)|bit(WORD_T))); // 移除单一含义的值字。
}
if (axis_command) { bit_false(value_words,(bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z)|bit(WORD_A)|bit(WORD_B)|bit(WORD_C))); } // 移除轴字。
if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [未使用的字]

// -------------------------------------------------------------------------------------
// 第 4 步：执行!!
// 假设所有错误检查已完成且不存在失败模式。我们只需根据执行顺序更新状态并执行区块。

// 初始化运动区块的计划数据结构。
plan_line_data_t plan_data;
plan_line_data_t *pl_data = &plan_data;
memset(pl_data,0,sizeof(plan_line_data_t)); // 将 pl_data 结构清零

// 拦截 jog 命令并完成有效 jog 命令的错误检查并执行。
// 注意：G-code 解析器状态未更新，除了位置以确保顺序 jog 目标正确计算。
// 在 jog 完成或被取消时，最终解析器位置将在 protocol_execute_realtime() 中更新。
if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    // 仅允许距离和单位模态命令及 G53 绝对覆盖命令。
// 注意：进给速率字和轴字检查在第 3 步中已完成。
    if (command_words & ~(bit(MODAL_GROUP_G3) | bit(MODAL_GROUP_G6 | bit(MODAL_GROUP_G0))) ) { FAIL(STATUS_INVALID_JOG_COMMAND) };
    if (!(gc_block.non_modal_command == NON_MODAL_ABSOLUTE_OVERRIDE || gc_block.non_modal_command == NON_MODAL_NO_ACTION)) { FAIL(STATUS_INVALID_JOG_COMMAND); }

// 初始化计划数据为当前主轴和冷却液模态状态。
    pl_data->spindle_speed = gc_state.spindle_speed;
    plan_data.condition = (gc_state.modal.spindle | gc_state.modal.coolant);

    uint8_t status = jog_execute(&plan_data, &gc_block);
    if (status == STATUS_OK) { memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); }
    return(status);
}

// 如果处于激光模式，根据当前和过去的解析器条件设置激光功率。
if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
    if ( !((gc_block.modal.motion == MOTION_MODE_LINEAR) || (gc_block.modal.motion == MOTION_MODE_CW_ARC) 
        || (gc_block.modal.motion == MOTION_MODE_CCW_ARC)) ) {
        gc_parser_flags |= GC_PARSER_LASER_DISABLE;
    }

// 任何带有轴字的运动模式都可以从主轴速度更新中传递。
// 注意：没有轴字的 G1 和 G0 会将 axis_command 设置为 none。G28/30 被故意省略。
// TODO：检查未进入计划的 M3 启用运动的同步条件。（零长度）。
    if (axis_words && (axis_command == AXIS_COMMAND_MOTION_MODE)) { 
        gc_parser_flags |= GC_PARSER_LASER_ISMOTION; 
    } else {
        // M3 恒定功率激光要求在没有运动的行中在 G1/2/3 运动模式状态和反之之间更新激光时进行计划同步。
        if (gc_state.modal.spindle == SPINDLE_ENABLE_CW) {
            if ((gc_state.modal.motion == MOTION_MODE_LINEAR) || (gc_state.modal.motion == MOTION_MODE_CW_ARC) 
                || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
                if (bit_istrue(gc_parser_flags,GC_PARSER_LASER_DISABLE)) { 
                    gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC; // 从 G1/2/3 运动模式改变。
                }
            } else {
                // 当从非 G1/2/3 运动模式更改为没有轴字的 G1 运动模式时。
                if (bit_isfalse(gc_parser_flags,GC_PARSER_LASER_DISABLE)) { 
                    gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC;
                }
            } 
        }
    }
}

// [0. 非特定/通用错误检查和杂项设置]:
// 注意：如果没有行号，值为零。
gc_state.line_number = gc_block.values.n;
pl_data->line_number = gc_state.line_number; // 记录供计划使用的数据。

// [1. 注释反馈 ]: 不支持

// [2. 设置进给速率模式 ]:
gc_state.modal.feed_rate = gc_block.modal.feed_rate;
if (gc_state.modal.feed_rate) { pl_data->condition |= PL_COND_FLAG_INVERSE_TIME; } // 设置供计划使用的条件标志。

// [3. 设置进给速率 ]:
gc_state.feed_rate = gc_block.values.f; // 始终复制该值。请参见进给速率错误检查。
pl_data->feed_rate = gc_state.feed_rate; // 记录供计划使用的数据。

// [4. 设置主轴速度 ]:
if ((gc_state.spindle_speed != gc_block.values.s) || bit_istrue(gc_parser_flags,GC_PARSER_LASER_FORCE_SYNC)) {
    if (gc_state.modal.spindle != SPINDLE_DISABLE) { 
        if (bit_isfalse(gc_parser_flags,GC_PARSER_LASER_ISMOTION)) {
            if (bit_istrue(gc_parser_flags,GC_PARSER_LASER_DISABLE)) {
                spindle_sync(gc_state.modal.spindle, 0.0);
            } else { spindle_sync(gc_state.modal.spindle, gc_block.values.s); }
        }
    }
    gc_state.spindle_speed = gc_block.values.s; // 更新主轴速度状态。
}
// 注意：对所有受限激光运动传递零主轴速度。
if (bit_isfalse(gc_parser_flags,GC_PARSER_LASER_DISABLE)) {
    pl_data->spindle_speed = gc_state.spindle_speed; // 记录供计划使用的数据。 
} // else { pl_data->spindle_speed = 0.0; } // 已初始化为零。

// [5. 选择工具 ]: 不支持。仅跟踪工具值。
gc_state.tool = gc_block.values.t;

// [6. 更换工具 ]: 不支持

// [7. 主轴控制 ]:
if (gc_state.modal.spindle != gc_block.modal.spindle) {
    // 在此区块启用时更新主轴控制并应用主轴速度。
    // 注意：所有主轴状态变化都是同步的，即使在激光模式下。此外，pl_data，
    // 而不是 gc_state，用于管理非激光运动的激光状态。
    spindle_sync(gc_block.modal.spindle, pl_data->spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;
}
pl_data->condition |= gc_state.modal.spindle; // 设置供计划使用的条件标志。

// [8. 冷却液控制 ]:
if (gc_state.modal.coolant != gc_block.modal.coolant) {
    // 注意：冷却液 M 代码是模态的。每行仅允许一个命令。但是，可以同时存在多个状态，而冷却液禁用将清除所有状态。
    coolant_sync(gc_block.modal.coolant);
    if (gc_block.modal.coolant == COOLANT_DISABLE) { gc_state.modal.coolant = COOLANT_DISABLE; }
    else { gc_state.modal.coolant |= gc_block.modal.coolant; }
}
pl_data->condition |= gc_state.modal.coolant; // 设置供计划使用的条件标志。

// [9. 启用/禁用进给速率或主轴覆盖 ]: 不支持。始终启用。

// [10. 停顿 ]:
if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }

// [11. 设置活动平面 ]:
gc_state.modal.plane_select = gc_block.modal.plane_select;

// [12. 设置长度单位 ]:
gc_state.modal.units = gc_block.modal.units;

// [13. 刀具半径补偿 ]: G41/42 不支持
// gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // 注意：由于始终禁用，因此不需要。

// [14. 刀具长度补偿 ]: G43.1 和 G49 支持。G43 不支持。
// 注意：如果支持 G43，其操作在执行上与 G43.1 并无不同。
// 错误检查步骤将简单地将偏移值加载到区块 XYZ 值数组的正确轴中。
if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // 表示更改。
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_CANCEL) { // G49
        gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] = 0.0;
    } // else G43.1
    if ( gc_state.tool_length_offset != gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] ) {
        gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
        system_flag_wco_change();
    }
}

// [15. 坐标系选择 ]:
if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,block_coord_system,N_AXIS*sizeof(float));
    system_flag_wco_change();
}

// [16. 设置路径控制模式 ]: G61.1/G64 不支持
// gc_state.modal.control = gc_block.modal.control; // 注意：始终默认。

// [17. 设置距离模式 ]:
gc_state.modal.distance = gc_block.modal.distance;

// [18. 设置回缩模式 ]: 不支持

// [19. 前往预定义位置、设置 G10 或设置轴偏移 ]:
switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
        settings_write_coord_data(coord_select,gc_block.values.ijk);
        // 如果当前激活，则更新系统坐标系。
        if (gc_state.modal.coord_select == coord_select) {
            memcpy(gc_state.coord_system,gc_block.values.ijk,N_AXIS*sizeof(float));
            system_flag_wco_change();
        }
        break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1:
        // 在回家之前移动到中间位置。遵循当前坐标系和偏移以及绝对和增量模式。
        pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // 设置快速运动条件标志。
        if (axis_command) { mc_line(gc_block.values.xyz, pl_data); }
        mc_line(gc_block.values.ijk, pl_data);
        memcpy(gc_state.position, gc_block.values.ijk, N_AXIS*sizeof(float));
        break;
    case NON_MODAL_SET_HOME_0:
        settings_write_coord_data(SETTING_INDEX_G28,gc_state.position);
        break;
    case NON_MODAL_SET_HOME_1:
        settings_write_coord_data(SETTING_INDEX_G30,gc_state.position);
        break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
        memcpy(gc_state.coord_offset,gc_block.values.xyz,sizeof(gc_block.values.xyz));
        system_flag_wco_change();
        break;
    case NON_MODAL_RESET_COORDINATE_OFFSET:
        clear_vector(gc_state.coord_offset); // 通过将偏移向量归零禁用 G92 偏移。
        system_flag_wco_change();
        break;
}

// [20. 运动模式 ]:
// 注意：命令 G10,G28,G30,G92 锁定并防止在运动模式中使用轴字。
// 仅在区块中有轴字或运动模式命令字时进入运动模式。
gc_state.modal.motion = gc_block.modal.motion;
if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
        uint8_t gc_update_pos = GC_UPDATE_POS_TARGET;
        if (gc_state.modal.motion == MOTION_MODE_LINEAR) {
            mc_line(gc_block.values.xyz, pl_data);
        } else if (gc_state.modal.motion == MOTION_MODE_SEEK) {
            pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // 设置快速运动条件标志。
            mc_line(gc_block.values.xyz, pl_data);
        } else if ((gc_state.modal.motion == MOTION_MODE_CW_ARC) || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
            mc_arc(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
                axis_0, axis_1, axis_linear, bit_istrue(gc_parser_flags,GC_PARSER_ARC_IS_CLOCKWISE));
        } else {
            // 注意：gc_block.values.xyz 是从 mc_probe_cycle 返回的，包含更新后的位置信息。
// 因此，在成功的探测周期之后，机器位置和返回值应相同。
#ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
            pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
#endif
            gc_update_pos = mc_probe_cycle(gc_block.values.xyz, pl_data, gc_parser_flags);
        }  

        // 对于解析器而言，位置现在 == 目标。实际上，
// 运动控制系统可能仍在处理动作，真实工具位置可能在任何中间位置。
        if (gc_update_pos == GC_UPDATE_POS_TARGET) {
            memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
        } else if (gc_update_pos == GC_UPDATE_POS_SYSTEM) {
            gc_sync_position(); // gc_state.position[] = sys_position
        } // == GC_UPDATE_POS_NONE
    }     
}

  // [21. 程序流程]:
// M0,M1,M2,M30: 执行非运行程序流程操作。在程序暂停期间，缓冲区可能会
// 重新填充，且只能通过循环开始运行命令恢复执行。
gc_state.modal.program_flow = gc_block.modal.program_flow;
if (gc_state.modal.program_flow) {
    protocol_buffer_synchronize(); // 在继续之前同步并完成所有剩余的缓冲运动。
    if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
        if (sys.state != STATE_CHECK_MODE) {
            system_set_exec_state_flag(EXEC_FEED_HOLD); // 使用进给保持进行程序暂停。
            protocol_execute_realtime(); // 执行暂停。
        }
    } else { // == PROGRAM_FLOW_COMPLETED
        // 程序完成后，只有部分 g 代码重置为某些默认值，具体根据
        // LinuxCNC 的程序结束描述和测试。只有模态组 [G 代码 1,2,3,5,7,12]
        // 和 [M 代码 7,8,9] 重置为 [G1,G17,G90,G94,G40,G54,M5,M9,M48]。其余模态组
        // [G 代码 4,6,8,10,13,14,15] 和 [M 代码 4,5,6] 以及模态字 [F,S,T,H] 不重置。
        gc_state.modal.motion = MOTION_MODE_LINEAR;
        gc_state.modal.plane_select = PLANE_SELECT_XY;
        gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
        gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
        // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // 不支持。
        gc_state.modal.coord_select = 0; // G54
        gc_state.modal.spindle = SPINDLE_DISABLE;
        gc_state.modal.coolant = COOLANT_DISABLE;
        // gc_state.modal.override = OVERRIDE_DISABLE; // 不支持。

        #ifdef RESTORE_OVERRIDES_AFTER_PROGRAM_END
            sys.f_override = DEFAULT_FEED_OVERRIDE;
            sys.r_override = DEFAULT_RAPID_OVERRIDE;
            sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;
        #endif

        // 执行坐标变化和主轴/冷却液停止。
        if (sys.state != STATE_CHECK_MODE) {
            if (!(settings_read_coord_data(gc_state.modal.coord_select, gc_state.coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
            system_flag_wco_change(); // 设置为立即刷新，以防有东西被更改。
            spindle_set_state(SPINDLE_DISABLE, 0.0);
            coolant_set_state(COOLANT_DISABLE);
        }
        report_feedback_message(MESSAGE_PROGRAM_END);
    }
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // 重置程序流程。
}

// TODO: 使用 % 来表示程序开始。

return(STATUS_OK);
}


/*
  不支持的功能：

  - 罐装循环
  - 刀具半径补偿
  - A,B,C 轴
  - 表达式求值
  - 变量
  - 覆盖控制（待定）
  - 刀具更换
  - 开关

   (*) 表示可选参数，通过 config.h 启用并重新编译
   组 0 = {G92.2, G92.3}（非模态：取消和重新启用 G92 偏移）
   组 1 = {G81 - G89}（运动模式：罐装循环）
   组 4 = {M1}（可选停止，忽略）
   组 6 = {M6}（刀具更换）
   组 7 = {G41, G42} 刀具半径补偿（支持 G40）
   组 8 = {G43} 刀具长度偏移（支持 G43.1/G49）
   组 9 = {M48, M49} 启用/禁用进给和速度覆盖开关
   组 10 = {G98, G99} 返回模式罐装循环
   组 13 = {G61.1, G64} 路径控制模式（支持 G61）
*/