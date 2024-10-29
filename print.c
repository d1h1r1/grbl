/*
  print.c - 格式化输出字符串的函数
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}


// 打印存储在 PGM 内存中的字符串
void printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    serial_write(c);
}


// void printIntegerInBase(unsigned long n, unsigned long base)
// {
// 	unsigned char buf[8 * sizeof(long)]; // 假定 8 位字符。
// 	unsigned long i = 0;
//
// 	if (n == 0) {
// 		serial_write('0');
// 		return;
// 	} 
// 
// 	while (n > 0) {
// 		buf[i++] = n % base;
// 		n /= base;
// 	}
// 
// 	for (; i > 0; i--)
// 		serial_write(buf[i - 1] < 10 ?
// 			'0' + buf[i - 1] :
// 			'A' + buf[i - 1] - 10);
// }


// 以十进制打印 uint8 变量。
void print_uint8_base10(uint8_t n)
{
  uint8_t digit_a = 0;
  uint8_t digit_b = 0;
  if (n >= 100) { // 100-255
    digit_a = '0' + n % 10;
    n /= 10;
  }
  if (n >= 10) { // 10-99
    digit_b = '0' + n % 10;
    n /= 10;
  }
  serial_write('0' + n);
  if (digit_b) { serial_write(digit_b); }
  if (digit_a) { serial_write(digit_a); }
}


// 以二进制打印 uint8 变量，具有所需的位数。
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits) {
  unsigned char buf[digits];
  uint8_t i = 0;

  for (; i < digits; i++) {
      buf[i] = n % 2;
      n /= 2;
  }

  for (; i > 0; i--)
      serial_write('0' + buf[i - 1]);
}


void print_uint32_base10(uint32_t n)
{
  if (n == 0) {
    serial_write('0');
    return;
  }

  unsigned char buf[10];
  uint8_t i = 0;

  while (n > 0) {
    buf[i++] = n % 10;
    n /= 10;
  }

  for (; i > 0; i--)
    serial_write('0' + buf[i - 1]);
}


void printInteger(long n)
{
  if (n < 0) {
    serial_write('-');
    print_uint32_base10(-n);
  } else {
    print_uint32_base10(n);
  }
}


// 通过立即转换为长整型将浮点数转换为字符串，该整数包含比浮点数更多的数字。
// 小数位数由计数器跟踪，可以由用户设置。然后有效地将整数转换为字符串。
// 注意：AVR 的 '%' 和 '/' 整数操作非常高效。位移加速
// 技术实际上只是稍微慢一点。通过艰难的方式发现了这一点。
void printFloat(float n, uint8_t decimal_places)
{
  if (n < 0) {
    serial_write('-');
    n = -n;
  }

  uint8_t decimals = decimal_places;
  while (decimals >= 2) { // 快速转换预期为 E0 到 E-4 的值。
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // 添加舍入因子。确保整个值的进位。

  // 反向生成数字并存储在字符串中。
  unsigned char buf[13];
  uint8_t i = 0;
  uint32_t a = (long)n;
  while (a > 0) {
    buf[i++] = (a % 10) + '0'; // 获取数字
    a /= 10;
  }
  while (i < decimal_places) {
     buf[i++] = '0'; // 填充小数点前的零（当 n < 1 时）
  }
  if (i == decimal_places) { // 如有需要，填充前导零。
    buf[i++] = '0';
  }

  // 打印生成的字符串。
  for (; i > 0; i--) {
    if (i == decimal_places) { serial_write('.'); } // 在正确位置插入小数点。
    serial_write(buf[i - 1]);
  }
}


// 用于 Grbl 中特殊变量类型的浮点值打印处理程序，定义在 config.h 中。
//  - CoordValue: 处理以英寸或毫米报告的所有位置或坐标值。
//  - RateValue: 处理以英寸或毫米报告的进给速率和当前速度。
void printFloat_CoordValue(float n) {
  if (bit_istrue(settings.flags, BITFLAG_REPORT_INCHES)) {
    printFloat(n * INCH_PER_MM, N_DECIMAL_COORDVALUE_INCH);
  } else {
    printFloat(n, N_DECIMAL_COORDVALUE_MM);
  }
}

void printFloat_RateValue(float n) {
  if (bit_istrue(settings.flags, BITFLAG_REPORT_INCHES)) {
    printFloat(n * INCH_PER_MM, N_DECIMAL_RATEVALUE_INCH);
  } else {
    printFloat(n, N_DECIMAL_RATEVALUE_MM);
  }
}

// 调试工具，用于在调用点打印剩余内存（以字节为单位）。
// 注意：除非使用，否则保持注释状态。此函数的部分始终会编译。
/*
void printFreeMemory()
{
  extern int __heap_start, *__brkval;
  uint16_t free;  // 最多 64k 值。
  free = (int) &free - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  printInteger((int32_t)free);
  printString(" ");
}
*/
