/*
  limits.c - 处理限位开关和执行回原点循环的代码
  Grbl 的一部分

  版权所有 (c) 2012-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：你可以在自由软件基金会发布的 GNU 通用公共许可证条款下重新分发和/或修改
  它，许可证版本为 3，或（根据你的选择）任何更高版本。

  Grbl 的发布是为了希望它能有用，
  但不提供任何担保；甚至没有关于
  适销性或适用于特定目的的隐含担保。有关详细信息，请参见
  GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/
  
#include "grbl.h"

// 回原点轴搜索距离倍增器。通过该值与循环移动的乘积计算得出。
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // 必须大于 1 以确保限位开关会被激活。
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // 必须大于 1 以确保限位开关被释放。
#endif

void limits_init()
{
  LIMIT_DDR &= ~(LIMIT_MASK); // 设置为输入引脚

  #ifdef DISABLE_LIMIT_PIN_PULL_UP
    LIMIT_PORT &= ~(LIMIT_MASK); // 正常低操作。需要外部下拉。
  #else
    LIMIT_PORT |= (LIMIT_MASK);  // 启用内部上拉电阻。正常高操作。
  #endif

  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    LIMIT_PCMSK |= LIMIT_MASK; // 启用引脚变化中断的特定引脚
    PCICR |= (1 << LIMIT_INT); // 启用引脚变化中断
  } else {
    limits_disable();
  }
  
  #ifdef ENABLE_SOFTWARE_DEBOUNCE
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDP0); // 设置超时时间约为 32 毫秒。
  #endif
}

// 禁用硬限位。
void limits_disable()
{
  LIMIT_PCMSK &= ~LIMIT_MASK;  // 禁用引脚变化中断的特定引脚
  PCICR &= ~(1 << LIMIT_INT);  // 禁用引脚变化中断
}

// 返回限位状态作为按位的 uint8 变量。每个位表示一个轴的限位，其中触发为 1，未触发为 0。应用反转掩码。轴由其在位位置中的编号定义，即 Z_AXIS 为 (1<<2) 或位 2，Y_AXIS 为 (1<<1) 或位 1。
uint8_t limits_get_state()
{
  uint8_t limit_state = 0;
  uint8_t pin = (LIMIT_PIN & LIMIT_MASK);
  #ifdef INVERT_LIMIT_PIN_MASK
    pin ^= INVERT_LIMIT_PIN_MASK;
  #endif
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
  if (pin) {  
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
      if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
    }
  }
  return(limit_state);
}

// 这是限位引脚变化中断，处理硬限位功能。抖动的限位开关可能会导致很多问题，如错误读数和多次中断调用。如果开关被触发，说明发生了错误，无论限位开关是否被解除，都应如此处理。因为 Arduino 微控制器在检测引脚变化时不会保留任何状态信息，所以无法可靠判断抖动引脚的状态。如果在 ISR 中轮询引脚，当开关抖动时，可能会错过正确的读数。
// 注意：不要将紧急停止开关连接到限位引脚，因为在回原点循环期间，此中断会被禁用，并且不会正确响应。根据用户请求或需要，可能会有紧急停止的特殊引脚输出，但通常建议将紧急停止开关直接连接到 Arduino 的复位引脚，因为这是最正确的方法。
#ifndef ENABLE_SOFTWARE_DEBOUNCE
  ISR(LIMIT_INT_vect) // 默认：限位引脚变化中断处理。 
  {
    // 如果已经处于报警状态或正在执行报警，则忽略限位开关。
    // 当处于报警状态时，Grbl 应该已经被重置，或者会强制重置，因此计划器和串口缓冲区中的所有待处理移动都会被清除，并且新的发送块将被锁定，直到进行回原点循环或杀死锁定命令。这允许用户在重置后，如果限位开关不断触发，则可以禁用硬限位设置并移动其轴。
    if (sys.state != STATE_ALARM) { 
      if (!(sys_rt_exec_alarm)) {
        #ifdef HARD_LIMIT_FORCE_STATE_CHECK
          // 检查限位引脚状态。 
          if (limits_get_state()) {
            mc_reset(); // 发起系统终止。
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // 指示硬限位关键事件
            uint8_t limit_state = limits_get_state();
            print_uint8_base10(limit_state);
            if (limit_state==8)
            {
              system_clear_exec_alarm();
              sys.state = STATE_IDLE;
            }
            
          }
        #else
          mc_reset(); // 发起系统终止。
          system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // 指示硬限位关键事件
        #endif
      }
    }
  }  
#else // 可选：软件去抖动限位引脚例程。
  // 在限位引脚变化时，启用看门狗定时器以产生短暂延迟。 
  ISR(LIMIT_INT_vect) { if (!(WDTCSR & (1<<WDIE))) { WDTCSR |= (1<<WDIE); } }
  ISR(WDT_vect) // 看门狗定时器 ISR
  {
    WDTCSR &= ~(1<<WDIE); // 禁用看门狗定时器。 
    if (sys.state != STATE_ALARM) {  // 如果已经处于报警状态则忽略。 
      if (!(sys_rt_exec_alarm)) {
        // 检查限位引脚状态。 
        uint8_t limit_state = limits_get_state();
        // limit_state = limit_state & 0xF7;
        if (limit_state) {
            mc_reset(); // 发起系统终止。
            print_uint8_base2_ndigit(limit_state, 8);
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // 指示硬限位关键事件
        }
      }  
    }
  }
#endif


 
// 回原点指定的循环轴，设置机器位置，并在完成后执行拉离动作。
// 回原点是一个特殊的运动案例，涉及快速无控制停止以定位限位开关的触发点。
// 快速停止由系统级轴锁掩码处理，该掩码防止步进算法执行步进脉冲。
// 回原点运动通常绕过正常操作中执行运动的过程。
// 注意：只有中止实时命令可以中断此过程。
// TODO：将限位引脚特定的调用移动到通用函数中，以提高可移植性。
void limits_go_home(uint8_t cycle_mask)
{
  if (sys.abort) { return; } // 如果已发出系统重置，则阻止。

  // 初始化用于回原点运动的计划数据结构。禁用主轴和冷却。
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t));
  pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
  pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;

  // 初始化用于回原点计算的变量。
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t step_pin[N_AXIS];
  float target[N_AXIS];
  float max_travel = 0.0;
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    // 初始化步进引脚掩码
    step_pin[idx] = get_step_pin_mask(idx);
    #ifdef COREXY
      if ((idx==A_MOTOR)||(idx==B_MOTOR)) { step_pin[idx] = (get_step_pin_mask(X_AXIS)|get_step_pin_mask(Y_AXIS)); }
    #endif

    if (bit_istrue(cycle_mask,bit(idx))) {
      // 基于 max_travel 设置目标。确保限位开关在搜索倍增器的影响下被激活。
      // 注意：settings.max_travel[] 存储为负值。
      max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
    }
  }

  // 设置搜索模式，以接近速率快速激活指定的 cycle_mask 限位开关。
  bool approach = true;
  float homing_rate = settings.homing_seek_rate;

  uint8_t limit_state, axislock, n_active_axis;
  do {

    system_convert_array_steps_to_mpos(target,sys_position);

    // 初始化并声明回原点例程所需的变量。
    axislock = 0;
    n_active_axis = 0;
    for (idx=0; idx<N_AXIS; idx++) {
      // 为活动轴设置目标位置并设置回原点速率的计算。
      if (bit_istrue(cycle_mask,bit(idx))) {
        n_active_axis++;
        #ifdef COREXY
          if (idx == X_AXIS) {
            int32_t axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
            sys_position[A_MOTOR] = axis_position;
            sys_position[B_MOTOR] = -axis_position;
          } else if (idx == Y_AXIS) {
            int32_t axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
            sys_position[A_MOTOR] = sys_position[B_MOTOR] = axis_position;
          } else {
            sys_position[Z_AXIS] = 0;
          }
        #else
          sys_position[idx] = 0;
        #endif
        // 根据循环掩码和回原点循环接近状态设置目标方向。
        // 注意：这样编译出来的代码比尝试过的任何其他实现都要小。
        if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
          if (approach) { target[idx] = -max_travel; }
          else { target[idx] = max_travel; }
        } else {
          if (approach) { target[idx] = max_travel; }
          else { target[idx] = -max_travel; }
        }
        // 将轴锁应用于本循环中活动的步进端口引脚。
        axislock |= step_pin[idx];
      }

    }
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] 调整以便每个轴都以回原点速率移动。
    sys.homing_axis_lock = axislock;

    // 执行回原点循环。计划器缓冲区应为空，以便启动回原点循环。
    pl_data->feed_rate = homing_rate; // 设置当前回原点速率。
    plan_buffer_line(target, pl_data); // 绕过 mc_line()。直接计划回原点运动。

    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; // 设置为执行回原点运动并清除现有标志。
    st_prep_buffer(); // 准备并填充段缓冲区，来源于新计划的块。
    st_wake_up(); // 启动运动
    do {
      if (approach) {
        // 检查限位状态。当它们发生变化时锁定循环轴。
        limit_state = limits_get_state();
        for (idx=0; idx<N_AXIS; idx++) {
          if (axislock & step_pin[idx]) {
            if (limit_state & (1 << idx)) {
              #ifdef COREXY
                if (idx==Z_AXIS) { axislock &= ~(step_pin[Z_AXIS]); }
                else { axislock &= ~(step_pin[A_MOTOR]|step_pin[B_MOTOR]); }
              #else
                axislock &= ~(step_pin[idx]);
              #endif
            }
          }
        }
        sys.homing_axis_lock = axislock;
      }

      st_prep_buffer(); // 检查并准备段缓冲区。注意：此操作应不超过 200 微秒。

      // 退出例程：在此循环中没有时间运行 protocol_execute_realtime()。
      if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
        uint8_t rt_exec = sys_rt_exec_state;
        // 回原点失败条件：在循环中发出重置。
        if (rt_exec & EXEC_RESET) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
        // 回原点失败条件：安全门已打开。
        if (rt_exec & EXEC_SAFETY_DOOR) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR); }
        // 回原点失败条件：拉离运动后限位开关仍被激活
        if (!approach && (limits_get_state() & cycle_mask)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF); }
        // 回原点失败条件：接近期间未找到限位开关。
        if (approach && (rt_exec & EXEC_CYCLE_STOP)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH); }
        if (sys_rt_exec_alarm) {
          mc_reset(); // 停止电机（如果正在运行）。
          protocol_execute_realtime();
          return;
        } else {
          // 拉离运动完成。禁用执行的 CYCLE_STOP。
          system_clear_exec_state_flag(EXEC_CYCLE_STOP);
          break;
        }
      }

    } while (STEP_MASK & axislock);

    st_reset(); // 立即强制停止步进电机并重置步进段缓冲区。
    delay_ms(settings.homing_debounce_delay); // 延迟以允许瞬态动态衰减。

    // 反转方向并重置回原点速率以进行定位循环。
    approach = !approach;

    // 在第一次循环后，回原点进入定位阶段。将搜索缩短至拉离距离。
    if (approach) {
      max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR;
      homing_rate = settings.homing_feed_rate;
    } else {
      max_travel = settings.homing_pulloff;
      homing_rate = settings.homing_seek_rate;
    }

  } while (n_cycle-- > 0);

  // 活动循环轴现在应已回原点，并且机器限位已被定位。默认情况下，Grbl 将机器空间定义为全部负值，正如大多数 CNC 一样。
// 由于限位开关可以位于轴的任一侧，因此检查并适当地设置轴的机器零点。同时，
// 为已回原点的轴限位开关设置拉离操作。这提供了一些初始的清离开关，并且应该有助于防止它们在启用硬限位时产生错误触发，或者当多个轴共享一个限位引脚时。
  int32_t set_axis_position;
  // 设置已回原点限位开关的机器位置。不要更新未回原点的轴。
  for (idx=0; idx<N_AXIS; idx++) {
    // 注意：settings.max_travel[] 存储为负值。
    if (cycle_mask & bit(idx)) {
      #ifdef HOMING_FORCE_SET_ORIGIN
        set_axis_position = 0;
      #else
        if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
          set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
        } else {
          set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
        }
      #endif

      #ifdef COREXY
        if (idx==X_AXIS) {
          int32_t off_axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
          sys_position[A_MOTOR] = set_axis_position + off_axis_position;
          sys_position[B_MOTOR] = set_axis_position - off_axis_position;
        } else if (idx==Y_AXIS) {
          int32_t off_axis_positionSTEP_CONTROL_NORMAL_OP = system_convert_corexy_to_x_axis_steps(sys_position);
          sys_position[A_MOTOR] = off_axis_position + set_axis_position;
          sys_position[B_MOTOR] = off_axis_position - set_axis_position;
        } else {
          sys_position[idx] = set_axis_position;
        }
      #else
        sys_position[idx] = set_axis_position;
      #endif

    }
  }
  sys.step_control = STEP_CONTROL_NORMAL_OP; // 将步进控制返回到正常操作。
}



// A轴回零
void a_go_home()
{
  protocol_buffer_synchronize();
  uint8_t cycle_mask = 1 << A_AXIS;
  uint8_t idx = 3;
  if (sys.abort) { return; } // 如果已发出系统重置，则阻止。

  // 初始化用于回原点运动的计划数据结构。禁用主轴和冷却。
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t));
  pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
  pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;

  // 初始化用于回原点计算的变量。
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t step_pin[N_AXIS];
  float target[N_AXIS];
  float max_travel = 0.0;
  // 初始化步进引脚掩码
  step_pin[idx] = get_step_pin_mask(idx);
  if (bit_istrue(cycle_mask, bit(idx)))
  {
      // 基于 max_travel 设置目标。确保限位开关在搜索倍增器的影响下被激活。
      // 注意：settings.max_travel[] 存储为负值。
      max_travel = max(max_travel, (-3) * settings.max_travel[idx]);
  }
 
  // 设置搜索模式，以接近速率快速激活指定的 cycle_mask 限位开关。
  bool approach = true;
  float homing_rate = settings.homing_seek_rate;

  uint8_t limit_state, axislock, n_active_axis;
  do {

    system_convert_array_steps_to_mpos(target,sys_position);

    // 初始化并声明回原点例程所需的变量。
    axislock = 0;
    n_active_axis = 0;
    for (idx=0; idx<N_AXIS; idx++) {
      // 为活动轴设置目标位置并设置回原点速率的计算。
      if (bit_istrue(cycle_mask,bit(idx))) {
        n_active_axis++;
        sys_position[idx] = 0;
        // 根据循环掩码和回原点循环接近状态设置目标方向。
        // 注意：这样编译出来的代码比尝试过的任何其他实现都要小。
        target[idx] = -max_travel;
        // 将轴锁应用于本循环中活动的步进端口引脚。
        axislock |= step_pin[idx];
      }

    }
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] 调整以便每个轴都以回原点速率移动。
    sys.homing_axis_lock = axislock;

    // 执行回原点循环。计划器缓冲区应为空，以便启动回原点循环。
    pl_data->feed_rate = homing_rate; // 设置当前回原点速率。
    plan_buffer_line(target, pl_data); // 绕过 mc_line()。直接计划回原点运动。

    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; // 设置为执行回原点运动并清除现有标志。
    st_prep_buffer(); // 准备并填充段缓冲区，来源于新计划的块。
    st_wake_up(); // 启动运动
    do {
      if (approach) {
          // 检查限位状态。当它们发生变化时锁定循环轴。
          limit_state = ~PINL & (1 << 5);
          if (axislock & step_pin[idx])
          {
              if (limit_state)
              {
                  axislock &= ~(step_pin[idx]);
              }
          }
          sys.homing_axis_lock = axislock;
      }

      st_prep_buffer(); // 检查并准备段缓冲区。注意：此操作应不超过 200 微秒。

      // 退出例程：在此循环中没有时间运行 protocol_execute_realtime()。
      if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
        uint8_t rt_exec = sys_rt_exec_state;
        // 回原点失败条件：在循环中发出重置。
        if (rt_exec & EXEC_RESET) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
        // 回原点失败条件：安全门已打开。
        if (rt_exec & EXEC_SAFETY_DOOR) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR); }
        // 回原点失败条件：拉离运动后限位开关仍被激活
        if (!approach && (limits_get_state() & cycle_mask)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF); }
        // 回原点失败条件：接近期间未找到限位开关。
        if (approach && (rt_exec & EXEC_CYCLE_STOP)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH); }
        if (sys_rt_exec_alarm) {
          mc_reset(); // 停止电机（如果正在运行）。
          protocol_execute_realtime();
          return;
        } else {
          // 拉离运动完成。禁用执行的 CYCLE_STOP。
          system_clear_exec_state_flag(EXEC_CYCLE_STOP);
          break;
        }
      }

    } while (STEP_MASK & axislock);

    st_reset(); // 立即强制停止步进电机并重置步进段缓冲区。
    delay_ms(settings.homing_debounce_delay); // 延迟以允许瞬态动态衰减。

    // 反转方向并重置回原点速率以进行定位循环。
    approach = !approach;

    // 在第一次循环后，回原点进入定位阶段。将搜索缩短至拉离距离。
    if (approach) {
      max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR;
      homing_rate = settings.homing_feed_rate;
    } else {
      max_travel = settings.homing_pulloff;
      homing_rate = settings.homing_seek_rate;
    }

  } while (n_cycle-- > 0);

  // 活动循环轴现在应已回原点，并且机器限位已被定位。默认情况下，Grbl 将机器空间定义为全部负值，正如大多数 CNC 一样。
// 由于限位开关可以位于轴的任一侧，因此检查并适当地设置轴的机器零点。同时，
// 为已回原点的轴限位开关设置拉离操作。这提供了一些初始的清离开关，并且应该有助于防止它们在启用硬限位时产生错误触发，或者当多个轴共享一个限位引脚时。
  sys_position[idx] = 0;

  sys.step_control = STEP_CONTROL_NORMAL_OP; // 将步进控制返回到正常操作。
}



// 执行软限位检查。仅从 mc_line() 调用。假设机器已回原点，
// 工作空间体积位于所有负空间中，系统处于正常操作状态。
// 注意：用于手动操作以限制在软限位体积内的行程。
void limits_soft_check(float *target)
{
  if (system_check_travel_limits(target)) {
    sys.soft_limit = true;
    // 如果循环处于活动状态，则强制进给保持。所有缓冲块都保证在
    // 工作空间体积内，因此仅需以受控方式停止，以避免位置丢失。完成后
    // 进入报警模式。
    if (sys.state == STATE_CYCLE) {
      system_set_exec_state_flag(EXEC_FEED_HOLD);
      do {
        protocol_execute_realtime();
        if (sys.abort) { return; }
      } while ( sys.state != STATE_IDLE );
    }
    mc_reset(); // 发出系统重置，确保主轴和冷却关闭。
    system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT); // 指示软限位的关键事件
    protocol_execute_realtime(); // 执行以进入关键事件循环和系统中止
    return;
  }
}
