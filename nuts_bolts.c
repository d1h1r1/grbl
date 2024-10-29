/*
  nuts_bolts.c - 共享函数
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

#include "grbl.h"

#define MAX_INT_DIGITS 8 // int32（和 float）中的最大数字位数

// 从字符串中提取浮点值。以下代码大致基于
// avr-libc strtod() 函数，由 Michael Stumpf 和 Dmitry Xmelkov 编写，以及许多可自由获取的
// 转换方法示例，但已针对 Grbl 进行了高度优化。对于已知的
// CNC 应用，典型的十进制值预计在 E0 到 E-4 范围内。
// 科学计数法在 G 代码中官方不支持，并且 'E' 字符在某些 CNC 系统中可能是
// G 代码单词。因此，不会识别 'E' 表示法。
// 注意：感谢 Radu-Eosif Mihailescu 识别出使用 strtod() 的问题。
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)
{
  char *ptr = line + *char_counter;
  unsigned char c;

  // 获取第一个字符并增加指针。假设行中没有空格。
  c = *ptr++;

  // 捕获初始的正/负号字符
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }

  // 将数字提取到快速整数中。跟踪小数的指数值。
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) { exp--; }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) { exp++; }  // 丢弃溢出位
      }
    } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }

  // 如果没有读取到数字，则返回。
  if (!ndigit) { return(false); };

  // 将整数转换为浮点数。
  float fval;
  fval = (float)intval;

  // 应用小数。对于预期范围内的 E0 到 E-4，
  // 不应执行超过两个浮点乘法。
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01;
      exp += 2;
    }
    if (exp < 0) {
      fval *= 0.1;
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    }
  }

  // 用正确的符号分配浮点值。
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }

  *char_counter = ptr - line - 1; // 将 char_counter 设置为下一个语句

  return(true);
}

// 非阻塞延迟函数，用于一般操作和挂起功能。
void delay_sec(float seconds, uint8_t mode)
{
 	uint16_t i = ceil(1000/DWELL_TIME_STEP*seconds);
	while (i-- > 0) {
		if (sys.abort) { return; }
		if (mode == DELAY_MODE_DWELL) {
			protocol_execute_realtime();
		} else { // DELAY_MODE_SYS_SUSPEND
		  // 仅执行 rt_system() 以避免嵌套挂起循环。
		  protocol_exec_rt_system();
		  if (sys.suspend & SUSPEND_RESTART_RETRACT) { return; } // 如果安全门重新打开则退出。
		}
		_delay_ms(DWELL_TIME_STEP); // 延迟 DWELL_TIME_STEP 增量
	}
}

// 延迟定义的毫秒数。编译器兼容性修复，_delay_ms()，
// 仅接受常量以便将来的编译器版本。
void delay_ms(uint16_t ms)
{
  while ( ms-- ) { _delay_ms(1); }
}

// 延迟定义的微秒数。编译器兼容性修复，_delay_us()，
// 仅接受常量以便将来的编译器版本。编写以便在较大延迟时更高效，
// 因为计数器在每次迭代中增加寄生时间。
void delay_us(uint32_t us)
{
  while (us) {
    if (us < 10) {
      _delay_us(1);
      us--;
    } else if (us < 100) {
      _delay_us(10);
      us -= 10;
    } else if (us < 1000) {
      _delay_us(100);
      us -= 100;
    } else {
      _delay_ms(1);
      us -= 1000;
    }
  }
}

// 简单的斜边计算函数。
float hypot_f(float x, float y) { return(sqrt(x*x + y*y)); }

float convert_delta_vector_to_unit_vector(float *vector)
{
  uint8_t idx;
  float magnitude = 0.0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (vector[idx] != 0.0) {
      magnitude += vector[idx]*vector[idx];
    }
  }
  magnitude = sqrt(magnitude);
  float inv_magnitude = 1.0/magnitude;
  for (idx=0; idx<N_AXIS; idx++) { vector[idx] *= inv_magnitude; }
  return(magnitude);
}

float limit_value_by_axis_maximum(float *max_value, float *unit_vec)
{
  uint8_t idx;
  float limit_value = SOME_LARGE_VALUE;
  for (idx=0; idx<N_AXIS; idx++) {
    if (unit_vec[idx] != 0) {  // 避免除以零。
      limit_value = min(limit_value,fabs(max_value[idx]/unit_vec[idx]));
    }
  }
  return(limit_value);
}
