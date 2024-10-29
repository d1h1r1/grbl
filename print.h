/*
  print.h - 格式化输出字符串的函数
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#ifndef print_h
#define print_h


void printString(const char *s);

void printPgmString(const char *s);

void printInteger(long n);

void print_uint32_base10(uint32_t n);

// 以十进制打印 uint8 变量。
void print_uint8_base10(uint8_t n);

// 以二进制打印 uint8 变量，具有所需的位数。
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits);

void printFloat(float n, uint8_t decimal_places);

// 用于 Grbl 中特殊变量类型的浮点值打印处理程序。
//  - CoordValue: 处理以英寸或毫米报告的所有位置或坐标值。
//  - RateValue: 处理以英寸或毫米报告的进给速率和当前速度。
void printFloat_CoordValue(float n);
void printFloat_RateValue(float n);

// 调试工具，用于在调用点打印剩余内存（以字节为单位）。否则不使用。
void printFreeMemory();

#endif
