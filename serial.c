/*
  serial.c - 通过串口发送和接收字节的低级函数
  Grbl 的一部分

  版权 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，
  无论是许可证的第 3 版，还是（根据您的选择）任何更高版本。

  Grbl 的发布希望能对您有用，
  但不提供任何担保；甚至不包括适销性或特定用途适用性的隐含担保。有关更多详细信息，请参见
  GNU 通用公共许可证。

  您应该已收到 GNU 通用公共许可证的副本
  随 Grbl 一起。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/


#include "grbl.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1) // 接收环形缓冲区的大小
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1) // 发送环形缓冲区的大小

uint8_t serial_rx_buffer[RX_RING_BUFFER]; // 接收缓冲区
uint8_t serial_rx_buffer_head = 0; // 接收缓冲区头部索引
volatile uint8_t serial_rx_buffer_tail = 0; // 接收缓冲区尾部索引

uint8_t serial_tx_buffer[TX_RING_BUFFER]; // 发送缓冲区
uint8_t serial_tx_buffer_head = 0; // 发送缓冲区头部索引
volatile uint8_t serial_tx_buffer_tail = 0; // 发送缓冲区尾部索引


// 返回 RX 串口缓冲区中可用的字节数。
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // 复制以限制对 volatile 的多次调用
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// 返回 RX 串口缓冲区中已使用的字节数。
// 注意：已废弃。除非在 config.h 中启用经典状态报告，否则不会使用。
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // 复制以限制对 volatile 的多次调用
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// 返回 TX 串口缓冲区中已使用的字节数。
// 注意：仅用于调试和确保没有 TX 瓶颈。
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // 复制以限制对 volatile 的多次调用
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  // 设置波特率
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // 关闭波特率倍增 - 仅在 Uno 上需要
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // 对于高波特率（如 115200），开启波特率倍增
  #endif
  UBRR0H = UBRR0_value >> 8; // 设置高字节
  UBRR0L = UBRR0_value; // 设置低字节

  // 启用接收、发送和接收完整字节的中断
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // 默认为 8 位，无奇偶校验，1 个停止位
}


// 向 TX 串口缓冲区写入一个字节。由主程序调用。
void serial_write(uint8_t data) {
  // 计算下一个头部索引
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // 等待缓冲区有空间
  while (next_head == serial_tx_buffer_tail) {
    // TODO: 重新构建 st_prep_buffer() 的调用，以便在长打印期间在这里执行。
    if (sys_rt_exec_state & EXEC_RESET) { return; } // 仅检查中止以避免无限循环。
  }

  // 存储数据并前进头部
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // 启用数据寄存器空中断以确保 TX 流传输正在运行
  UCSR0B |=  (1 << UDRIE0);
}


// 数据寄存器空中断处理程序
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // 临时的 serial_tx_buffer_tail（以优化 volatile）

  // 从缓冲区发送一个字节
  UDR0 = serial_tx_buffer[tail];

  // 更新尾部位置
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // 关闭数据寄存器空中断以停止 TX 流传输，如果这结束了传输
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}


// 获取串口读取缓冲区中的第一个字节。由主程序调用。
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // 临时的 serial_rx_buffer_tail（以优化 volatile）
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA; // 无数据可读
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data; // 返回读取的数据
  }
}


ISR(SERIAL_RX)
{
  uint8_t data = UDR0; // 从接收数据寄存器读取数据
  uint8_t next_head;

  // 从串行流中直接获取实时命令字符。这些字符
  // 不会传递到主缓冲区，而是设置系统状态标志位以便实时执行。
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // 调用运动控制重置例程。
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // 设置为真
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // 设置为真
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // 设置为真
    default :
      if (data > 0x7F) { // 实时控制字符仅为扩展 ASCII。
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // 设置为真
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // 阻止其他状态调用运动取消。
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break; // 调试报告
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
        }
        // 丢弃任何未找到的扩展 ASCII 字符，不将其传递到串口缓冲区。
      } else { // 将字符写入缓冲区
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // 将数据写入缓冲区，除非已满。
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}


void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head; // 重置读取缓冲区
}
