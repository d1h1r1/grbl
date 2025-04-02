/*
  spindle_control.c - 主轴控制方法
  Grbl的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon, Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以根据 GNU 通用公共许可证的条款重新分发和/或修改
  它，该许可证由自由软件基金会发布，许可证的版本为第 3 版，或
  （根据您的选择）任何更高版本。

  Grbl 的发行目的是希望它对您有用，
  但不提供任何担保；甚至没有对适销性或特定目的适用性的暗示担保。有关更多详细信息，请参阅
  GNU 通用公共许可证。

  您应该已经收到了一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

static float pwm_gradient; // 预先计算的值，用于加速转速到PWM的转换。

uint16_t crc_chk_value(uint8_t *data_value, uint8_t length)
{
  uint16_t crc_value=0xFFFF;
  int i; 
  while(length--){ 
    crc_value ^= *data_value++;
    for(i=0;i<8;i++){ 
      if(crc_value&0x0001){
        crc_value=( crc_value>>1)^0xA001;
      }else{
        crc_value= crc_value>>1; 
      }
    }
  }
  return(crc_value); 
}

void float2Bytes(uint8_t bytes_temp[4],float float_variable){
  union {
    float a;
    uint8_t bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

void control485(uint8_t *sendData){
    uint8_t bytes[2];
    uint16_t value = crc_chk_value(sendData, 6);
    memcpy(bytes, &value, sizeof(value));
    for(uint8_t i=0; i < 6; i++){
      serial2_write(sendData[i]);
    }
    serial2_write(bytes[0]);
    serial2_write(bytes[1]);
    delay_ms(50);
}

void spindle_init()
{
  // 配置可变主轴PWM和使能引脚（如需要）。
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // 配置为PWM输出引脚。
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // 配置PWM输出比较定时器
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_OCRA_REGISTER = SPINDLE_OCRA_TOP_VALUE; // 设置16位快速PWM模式的顶部值
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // 配置为输出引脚。
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT | 1<<(SPINDLE_DIRECTION_BIT + 1)); // 配置为输出引脚。

  pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  spindle_stop();
}

uint8_t spindle_get_state()
{
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT)) && (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT))) {
  #else
    if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT)) && (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT))) {
  #endif
    if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
    else { return(SPINDLE_STATE_CW); }
  }
	return(SPINDLE_STATE_DISABLE);
}


// 禁用主轴，并在启用PWM可变主轴速度时将PWM输出设置为零。
// 由各种主程序和ISR例程调用。保持例程简短、快速和高效。
// 由 spindle_init()、spindle_set_speed()、spindle_set_state() 和 mc_reset() 调用。
void spindle_stop()
{
  // 发送485停机指令
  uint8_t sendData[] = {0x01, 0x06, 0x13, 0x00, 0x00, 0x00};
  control485(sendData);
  SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // 禁用PWM。输出电压为零。
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // 设置引脚为高
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // 设置引脚为低
  #endif
}

// 设置主轴速度的PWM输出和使能引脚（如果配置）。由 spindle_set_state() 和步进 ISR 调用。
// 保持例程简短和高效。
void spindle_set_speed(uint16_t pwm_value)
{
  SPINDLE_OCR_REGISTER = pwm_value; // 设置PWM输出水平。
  if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // 禁用PWM。输出电压为零。
  } else {
    SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // 确保启用PWM输出。
  }
}

// 由 spindle_set_state() 和步进段生成器调用。保持例程简短和高效。
uint16_t spindle_compute_pwm_value(float rpm) // Mega2560 PWM寄存器为16位。
{
  uint16_t pwm_value;
  rpm *= (0.010*sys.spindle_speed_ovr); // 按主轴速度覆盖值缩放。
  // 根据转速最大/最小设置和编程转速计算PWM寄存器值。
  if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
    // 无法实现PWM范围。设置简单的开/关主轴控制引脚状态。
    sys.spindle_speed = settings.rpm_max;
    pwm_value = SPINDLE_PWM_MAX_VALUE;
  } else if (rpm <= settings.rpm_min) {
    if (rpm == 0.0) { // S0 禁用主轴
      sys.spindle_speed = 0.0;
      pwm_value = SPINDLE_PWM_OFF_VALUE;
    } else { // 设置最低PWM输出
      sys.spindle_speed = settings.rpm_min;
      pwm_value = SPINDLE_PWM_MIN_VALUE;
    }
  } else { 
    // 使用线性主轴速度模型计算中间PWM值。
    // 注意：如果需要，可以在此安装非线性模型，但请保持其非常轻量。
    sys.spindle_speed = rpm;
    pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
  }
  return(pwm_value);
}

// 立即通过PWM设置主轴运行状态、方向和转速（如果启用）。
// 由g-code解析器 spindle_sync()、停车回撤和恢复、g-code程序结束、
// 睡眠和主轴停止覆盖调用。
void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // 在中止期间阻塞。
  if (state == SPINDLE_DISABLE) { // 停止或设置主轴方向和转速。
    sys.spindle_speed = 0.0;
    spindle_stop();
  
  } else {
    if (state == SPINDLE_ENABLE_CW) {
      // 发送485顺时针反转指令
      uint8_t sendData[] = {0x01, 0x06, 0x13, 0x00, 0x00, 0x02};
      control485(sendData);

      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      SPINDLE_DIRECTION_PORT |= (1<<(SPINDLE_DIRECTION_BIT + 1));
    } else {
      // 发送485逆时针正转指令
      uint8_t sendData[] = {0x01, 0x06, 0x13, 0x00, 0x00, 0x01};
      control485(sendData);

      SPINDLE_DIRECTION_PORT &= ~(1<<(SPINDLE_DIRECTION_BIT+1));
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }
    // 注意：假设所有调用此函数的时机都是当Grbl不在移动或必须保持关闭状态。
    if (settings.flags & BITFLAG_LASER_MODE) { 
      if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: 可能需要 rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
    }
    // 485转速控制
    uint16_t hz = rpm / 60 * 100;
    uint8_t bytes[2];
    memcpy(bytes, &hz, sizeof(hz));
    uint8_t sendData[] = {0x01, 0x06, 0x13, 0x01, bytes[1], bytes[0]};
    control485(sendData);

    spindle_set_speed(spindle_compute_pwm_value(rpm));

		#ifdef INVERT_SPINDLE_ENABLE_PIN
			SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
		#else
			SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
		#endif   
  }
  sys.report_ovr_counter = 0; // 设置为立即报告更改
}

// G-code解析器设置主轴状态的入口点。强制进行规划缓冲区同步，如果正在进行中止或检查模式则退出。
void spindle_sync(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // 清空规划缓冲区以确保在编程时设置主轴。
  spindle_set_state(state, rpm);
}
