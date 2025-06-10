#include "grbl.h"

#define LED_PORT PORTD
#define LED_DDR  DDRD
#define LED_PIN  PD2  // D6 对应 Mega2560 的 PH3

void sendByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & (1 << (7 - i))) {
      // 1 bit: 高电平 ~0.8us，低电平 ~0.45us
      LED_PORT |= (1 << LED_PIN);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n nop\n nop\n nop\n"
      );
      LED_PORT &= ~(1 << LED_PIN);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n"
      );
    } else {
      // 0 bit: 高电平 ~0.4us，低电平 ~0.85us
      LED_PORT |= (1 << LED_PIN);
      asm volatile (
        "nop\n nop\n nop\n nop\n"
      );
      LED_PORT &= ~(1 << LED_PIN);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
    }
  }
}

void sendColor(uint8_t r, uint8_t g, uint8_t b) {
  sendByte(g);
  sendByte(r);
  sendByte(b);
}

void ws2812b_init_once() {
  DDRD |= (1 << 0); // 将其配置为输出引脚。
  PORTD |= (1<<0);  // 设置引脚为高，继电器默认闭合
  DDRD |= (1 << 2); // 将其配置为输出引脚。
  PORTD |= (1<<2);  // 设置引脚为高，继电器默认闭合

  // LED_DDR |= (1 << LED_PIN);  // 设置 D6 为输出
  // LED_PORT |= (1 << LED_PIN);
  // cli(); // 临时关闭中断（发送期间必须）
  // sendColor(255, 0, 0);  // 显示绿色
  // sendColor(0, 255, 0);  // 显示绿色
  // sendColor(0, 0, 255);  // 显示绿色
  // sei(); // 恢复中断
  // _delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
}