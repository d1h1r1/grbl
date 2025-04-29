// #include "grbl.h"

// void sendByte(uint8_t b) {
//   for (uint8_t i = 0; i < 8; i++) {
//     if (b & (1 << (7 - i))) {
//       // 1 bit: 高电平 ~0.8us，低电平 ~0.45us
//       LED_PORT |= (1 << LED_PIN);
//       asm volatile (
//         "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
//         "nop\n nop\n nop\n nop\n nop\n"
//       );
//       LED_PORT &= ~(1 << LED_PIN);
//       asm volatile (
//         "nop\n nop\n nop\n nop\n nop\n"
//       );
//     } else {
//       // 0 bit: 高电平 ~0.4us，低电平 ~0.85us
//       LED_PORT |= (1 << LED_PIN);
//       asm volatile (
//         "nop\n nop\n nop\n nop\n"
//       );
//       LED_PORT &= ~(1 << LED_PIN);
//       asm volatile (
//         "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
//         "nop\n nop\n"
//       );
//     }
//   }
// }

// void sendColor(uint8_t r, uint8_t g, uint8_t b) {
//   sendByte(g);
//   sendByte(r);
//   sendByte(b);
// }

// void ws2812b_init_once() {
//   LED_DDR |= (1 << LED_PIN);  // 设置 D6 为输出

//   cli(); // 临时关闭中断（发送期间必须）
//   for(int i=0; i<= 60; i++){
//     sendColor(255, 0, 0);  // 显示绿色
//   }
//   sei(); // 恢复中断
// }
