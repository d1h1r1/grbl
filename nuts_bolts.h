/*
  nuts_bolts.h - 共享定义、变量和函数的头文件
  Grbl的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改
  它，许可证的版本为 3，或者（根据您的选择）任何后续版本。

  Grbl 以希望它有用的方式分发，
  但不提供任何保证；甚至没有对
  适销性或适合特定目的的隐含保证。请参见
  GNU 通用公共许可证的更多详细信息。

  您应该已收到 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

#ifndef nuts_bolts_h
#define nuts_bolts_h

#define false 0
#define true 1

#define SOME_LARGE_VALUE 1.0E+38

// 轴数组索引值。必须从 0 开始并连续。
#define N_AXIS 4 // 轴的数量
#define X_AXIS 0 // 轴索引值。
#define Y_AXIS 1
#define Z_AXIS 2
#define A_AXIS 3
//#define B_AXIS 4
//#define C_AXIS 5

// CoreXY 电机分配。请勿更改。
// 注意：如果更改了 A 和 B 电机轴的绑定，会影响 CoreXY 方程。
#ifdef COREXY
 #define A_MOTOR X_AXIS // 必须是 X_AXIS
 #define B_MOTOR Y_AXIS // 必须是 Y_AXIS
#endif

// 转换
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#define TICKS_PER_MICROSECOND (F_CPU/1000000)

#define DELAY_MODE_DWELL       0
#define DELAY_MODE_SYS_SUSPEND 1

// 有用的宏
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(float)*N_AXIS))

// 位域和掩码宏
#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// 从字符串读取浮点值。line 指向输入缓冲区，char_counter
// 是指向行中当前字符的索引器，而 float_ptr 是
// 指向结果变量的指针。成功时返回 true
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// 非阻塞延迟函数，用于一般操作和挂起功能。
void delay_sec(float seconds, uint8_t mode);

// 延迟定义的毫秒数。编译器兼容性修复，_delay_ms()。
void delay_ms(uint16_t ms);

// 延迟定义的微秒数。编译器兼容性修复，_delay_us()。
void delay_us(uint32_t us);

// 计算斜边，避免 avr-gcc 的臃肿版本和额外的错误检查。
float hypot_f(float x, float y);

float convert_delta_vector_to_unit_vector(float *vector);
float limit_value_by_axis_maximum(float *max_value, float *unit_vec);

#endif
