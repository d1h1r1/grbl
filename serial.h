/*
  serial.c - 通过串口发送和接收字节的低级函数
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证进行再发行和/或修改，
  该许可证的版本为 3，或（根据您的选择）任何后续版本。

  Grbl 的分发是出于它会有用的希望，
  但没有任何担保；甚至没有适销性或适合特定目的的隐含担保。有关更多详细信息，请参见
  GNU 通用公共许可证。

  您应该已收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/

#ifndef serial_h
#define serial_h

#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 510
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 255
#endif

#define SERIAL_NO_DATA 0xff

void serial_init();
void serial1_init();
void serial2_init();

// 将一个字节写入 TX 串口缓冲区。由主程序调用。
void serial_write(uint8_t data);
uint8_t serial_write_bytes(const uint8_t* data, uint8_t length);
void serial2_write(uint8_t data);
void serial1_write(uint8_t data);

// 获取串口读取缓冲区中的第一个字节。由主程序调用。
uint8_t serial_read();
uint8_t serial1_read();
uint8_t serial1_read_bytes(uint8_t* buffer, uint8_t length);
uint8_t serial2_read();

// 重置并清空读取缓冲区中的数据。由急停和重置使用。
void serial_reset_read_buffer();

// 返回 RX 串口缓冲区中可用的字节数。
uint8_t serial_get_rx_buffer_available();

// 返回 RX 串口缓冲区中已用的字节数。
// 注意：已弃用。除非在 config.h 中启用经典状态报告，否则不使用。
uint8_t serial_get_rx_buffer_count();
uint8_t serial1_get_rx_buffer_count();
// 返回 TX 串口缓冲区中已用的字节数。
// 注意：除调试和确保没有 TX 瓶颈外不使用。
uint8_t serial_get_tx_buffer_count();
void clearSerial1BufferHard();
#endif
