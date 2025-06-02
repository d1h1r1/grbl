#include "grbl.h"

// 定义软件I2C引脚 (使用PD2/PD3 - Arduino引脚2/3)
#define SDA_PIN 2    // PD2
#define SCL_PIN 3    // PD3
#define TM601_ADDR 0x40

// 寄存器定义
#define SDA_PORT PORTE
#define SDA_DDR DDRE
#define SDA_PIN_REG PINE
#define SDA_BIT 4
#define SCL_PORT PORTE
#define SCL_DDR DDRE
#define SCL_BIT 5

// 精确延时函数 (针对16MHz时钟优化)
static inline void I2C_Delay(void) __attribute__((always_inline));
static inline void I2C_Delay(void) {
  __asm__ __volatile__ ("nop\nnop\nnop\nnop\nnop"); // 约0.3us延时
}

// 自定义map函数实现
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 初始化I2C总线
void I2C_Init() {
  SCL_DDR |= (1 << SCL_BIT);  // SCL设为输出
  SDA_DDR |= (1 << SDA_BIT);  // SDA设为输出
  SCL_PORT |= (1 << SCL_BIT);  // SCL高电平
  SDA_PORT |= (1 << SDA_BIT);  // SDA高电平
  _delay_us(100);      // 总线稳定时间
}

// 产生起始条件
void I2C_Start() {
  SDA_PORT |= (1 << SDA_BIT);   // SDA高
  SCL_PORT |= (1 << SCL_BIT);   // SCL高
  I2C_Delay();
  SDA_PORT &= ~(1 << SDA_BIT);  // SDA低
  I2C_Delay();
  SCL_PORT &= ~(1 << SCL_BIT);   // SCL低
  I2C_Delay();
}

// 产生停止条件
void I2C_Stop() {
  SDA_PORT &= ~(1 << SDA_BIT);   // SDA低
  SCL_PORT &= ~(1 << SCL_BIT);   // SCL低
  I2C_Delay();
  SCL_PORT |= (1 << SCL_BIT);    // SCL高
  I2C_Delay();
  SDA_PORT |= (1 << SDA_BIT);    // SDA高
  I2C_Delay();
}

// 发送一个字节并返回ACK
bool I2C_Write(uint8_t data) {
  for(uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    if(data & mask) {
      SDA_PORT |= (1 << SDA_BIT);   // SDA高
    } else {
      SDA_PORT &= ~(1 << SDA_BIT);  // SDA低
    }
    I2C_Delay();
    SCL_PORT |= (1 << SCL_BIT);     // SCL高
    I2C_Delay();
    SCL_PORT &= ~(1 << SCL_BIT);    // SCL低
  }
  
  // 切换SDA为输入检测ACK
  SDA_DDR &= ~(1 << SDA_BIT);       // SDA设为输入
  SDA_PORT |= (1 << SDA_BIT);       // 上拉使能
  I2C_Delay();
  SCL_PORT |= (1 << SCL_BIT);       // SCL高
  I2C_Delay();
  bool ack = !(SDA_PIN_REG & (1 << SDA_BIT)); // 读取ACK
  SCL_PORT &= ~(1 << SCL_BIT);      // SCL低
  
  // 恢复SDA为输出
  SDA_DDR |= (1 << SDA_BIT);        // SDA设为输出
  return ack;
}

// 读取一个字节
uint8_t I2C_Read(bool ack) {
  uint8_t data = 0;
  SDA_DDR &= ~(1 << SDA_BIT);       // SDA设为输入
  SDA_PORT |= (1 << SDA_BIT);       // 上拉使能
  
  for(uint8_t i = 0; i < 8; i++) {
    SCL_PORT |= (1 << SCL_BIT);     // SCL高
    I2C_Delay();
    data = (data << 1) | ((SDA_PIN_REG >> SDA_BIT) & 0x01);
    SCL_PORT &= ~(1 << SCL_BIT);    // SCL低
    I2C_Delay();
  }
  
  // 发送ACK/NACK
  SDA_DDR |= (1 << SDA_BIT);        // SDA设为输出
  if(ack) {
    SDA_PORT &= ~(1 << SDA_BIT);    // SDA低(ACK)
  } else {
    SDA_PORT |= (1 << SDA_BIT);     // SDA高(NACK)
  }
  I2C_Delay();
  SCL_PORT |= (1 << SCL_BIT);       // SCL高
  I2C_Delay();
  SCL_PORT &= ~(1 << SCL_BIT);      // SCL低
  
  return data;
}

// TM601写入寄存器
void TM601_WriteReg(uint8_t reg, uint8_t val) {
  I2C_Start();
  I2C_Write(TM601_ADDR << 1);    // 写地址
  I2C_Write(reg);                // 寄存器地址
  I2C_Write(val);                // 数据
  I2C_Stop();
  _delay_us(50);         // 写周期等待
}

// TM601读取数据
void TM601_ReadData(uint8_t *buf, uint8_t len) {
  I2C_Start();
  I2C_Write(TM601_ADDR << 1);    // 写地址
  I2C_Write(0x00);               // 起始寄存器
  I2C_Start();
  I2C_Write((TM601_ADDR << 1)|1); // 读地址
  
  for(uint8_t i = 0; i < len; i++) {
    buf[i] = I2C_Read(i == (len-1)); // 最后一个字节发NACK
  }
  
  I2C_Stop();
}

// 带滤波的数据读取
uint8_t ReadFiltered() {
  static uint8_t filterBuf[5] = {0};
  static uint8_t index = 0;
  uint8_t rawData[4];
  
  TM601_ReadData(rawData, 4);
  uint8_t current = map(rawData[0], 0, 255, 0, 81);
  
  // 移动平均滤波
  filterBuf[index] = current;
  index = (index + 1) % 5;
  
  uint16_t sum = 0;
  for(uint8_t i = 0; i < 5; i++) {
    sum += filterBuf[i];
  }
  
  return sum / 5;
}

void getDepth_init() {
//   Serial.begin(115200);
  I2C_Init();
  
  // 检测设备是否存在
  I2C_Start();
  bool ack = I2C_Write(TM601_ADDR << 1);
  I2C_Stop();
  
//   if(!ack) {
//     Serial.println("Device not found!");
//     while(1);
//   }
  
//   Serial.println("TM601 Ready");
}

// void loop() {
//   uint8_t level = ReadFiltered();
//   Serial.print("Level: ");
//   Serial.print(level);
//   Serial.println(" mm");
//   delay(200);
// }