/*
  protocol.c - 控制 Grbl 的执行协议和流程
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以按照自由软件基金会发布的 GNU 通用公共许可证的条款，
  对其进行再分发和/或修改，许可证可以是第 3 版，也可以是（由您选择）任何
  以后的版本。

  Grbl 的发布希望它有用，
  但不提供任何保证；甚至没有默示的
  适销性或特定用途的适用性保证。详情请参阅
  GNU 通用公共许可证。

  您应该已经收到了一份随 Grbl 一起发布的 GNU 通用公共许可证。
  如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 定义行标志。包括注释类型跟踪和行溢出检测。
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.

static void protocol_exec_rt_suspend();

/*
  GRBL 主循环：
*/
void protocol_main_loop()
{
  // 执行一些机器检查以确保一切正常。
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limits_get_state()) {
        sys.state = STATE_ALARM; // 确保警报状态处于激活状态。
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
  // 在重置、错误或初始上电后检查并报告警报状态。
  // 注意：睡眠模式禁用步进驱动器，位置无法保证。
  // 将睡眠状态重新初始化为警报模式，以确保用户复位或确认。
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // 确保设置警报状态。
  } else {
    // 检查安全门是否打开。
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // 进入安全门模式。应返回为空闲状态。
    }
    // 一切正常！
    system_execute_startup(line); // 执行启动脚本。
  }

  // ---------------------------------------------------------------------------------
  // 主循环！在系统中止时，返回 main() 以重置系统。
  // 此处也是 Grbl 空闲等待任务的地方。
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // 处理一行传入的串行数据，当数据可用时进行处理。
    // 通过删除空格和注释并将所有字母大写来进行初步过滤。
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // 到达行末

        protocol_execute_realtime(); // 运行时命令检查点。
        if (sys.abort) { return; } // 系统中止时返回调用函数

        line[char_counter] = 0; // 设置字符串结束字符。
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // 直接执行一行格式化输入，并报告执行状态。
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // 报告行溢出错误。
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // 空行或注释行。用于同步。
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl '$' 系统命令
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // 其他情况均为 gcode。如果处于警报或 jog 模式，则阻止。
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // 解析并执行 g-code 块。
          report_status_message(gc_execute_line(line));
        }

        // 重置下一行的跟踪数据。
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // 丢弃所有（除 EOL 外）的注释字符和溢出字符。
          if (c == ')') {
            // 结束 '()' 注释。恢复允许的行。
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // 丢弃空白和控制字符
          } else if (c == '/') {
            // 不支持删除块。忽略字符。
            // 注意：如支持，只需检查系统是否启用删除块。
          } else if (c == '(') {
            // 启用注释标志，忽略所有字符，直到 ')' 或 EOL。
            // 注意：这不完全符合 NIST 定义，但目前已经足够。
            // 将来我们可以仅删除注释内的内容，但保留注释控制字符，
            // 这样 g-code 解析器可以进行错误检查。
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // 注意：';' 注释到 EOL 是 LinuxCNC 定义的，非 NIST 标准。
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: 添加 '%' 特性
          // } else if (c == '%') {
            // 不支持程序起止百分号。
            // 注意：可以用于告知 Grbl 程序的运行状态（程序运行 vs 手动输入），
            // 在程序期间，系统将自动开始执行所有任务直到下一个 '%' 符号。
            // 这有助于解决某些任务清空规划器缓冲区以按时执行的问题。
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // 检测到行缓冲区溢出并设置标志。
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // 转换小写字母为大写
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
              
    #ifdef SLEEP_ENABLE
      // Check for sleep conditions and execute auto-park, if timeout duration elapses.
      sleep_check();    
    #endif
  }

  return; /* Never reached */
}


// 阻塞直到所有缓存的步骤执行完毕或处于循环状态。在同步调用期间可配合进给暂停使用，
// 以便在需要时生效。同时等待干净的循环结束。
void protocol_buffer_synchronize()
{
  // 如果系统已排队，确保循环恢复（如果自动启动标志存在）。
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // 检查并执行运行时命令
    if (sys.abort) { return; } // 检查系统中止
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// 自动循环启动在有运动准备执行且主程序未主动解析命令时触发。
// 注意：此函数仅从主循环、缓冲区同步和 mc_line() 中调用，
// 并在以下情况之一存在时执行：没有更多块发送（即流已完成，单命令），
// 需要等待缓冲区中的运动执行时调用缓冲区同步，或者计划缓冲区已满并准备就绪。
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // 检查缓冲区中是否有任何块。
    system_set_exec_state_flag(EXEC_CYCLE_START); // 如果有，则执行它们！
  }
}


// This function is the general interface to Grbl's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}


// 当需要时执行运行时命令。此函数主要作为 Grbl 的状态机，
// 控制 Grbl 提供的各种实时功能。
// 注意：除非完全了解，否则不要更改此处！
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // 临时变量以避免多次调用易变数据。
  rt_exec = sys_rt_exec_alarm; // 复制易变的 sys_rt_exec_alarm。
  if (rt_exec) { // 仅在任意标志位为真时进入
    // 系统报警。由于严重错误导致一切已关闭。向用户报告错误来源。
    // 如果严重，Grbl 进入无限循环禁用系统，直到系统重置/中止。
    sys.state = STATE_ALARM; // 设置系统报警状态
    report_alarm_message(rt_exec);
    // 在发生严重事件标志时暂停所有内容。目前硬限位和软限位标记此项。
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // 禁用现有的重置
      do {
        // 阻止所有操作，除非重置或状态报告，直到用户发出重置或断电。
        // 硬限位通常在无人监控或未注意时发生，给予用户和 GUI 时间执行必要的操作。
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // 清除报警
  }

  rt_exec = sys_rt_exec_state; // 复制易变的 sys_rt_exec_state。
  if (rt_exec) {

    // 执行系统中止。
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // 仅在此处设置为 true。
      return; // 仅退出即可。
    }

    // 执行并串行打印状态
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    // 注意：一旦暂停启动，系统立即进入挂起状态，阻止所有主程序进程，直到重置或恢复。
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

      // 状态检查是否允许暂停方式。
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
      
        // 如果处于 CYCLE 或 JOG 状态，立即启动运动暂停。
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { // 若已暂停则阻止。
            st_update_plan_block_parameters(); // 通知步进模块重新计算暂停减速。
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; // 激活标志进入挂起状态。
            if (sys.state == STATE_JOG) { // 若正在执行 JOG 操作，暂停事件会取消该操作，除非休眠。
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        // 若处于空闲，Grbl 无运动，仅表示挂起状态且暂停已完成。
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        // 通过减速执行运动取消并返回空闲。主要用于探测循环，以停止并取消余下运动。
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // MOTION_CANCEL 仅在 CYCLE 状态发生，但 HOLD 和 SAFETY_DOOR 可能在此之前启动以暂停 CYCLE。
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // 状态为 STATE_CYCLE。
        }

        // 如果需要，通过减速执行进给暂停，然后挂起系统。
        if (rt_exec & EXEC_FEED_HOLD) {
          // 阻止 SAFETY_DOOR、JOG 和 SLEEP 状态切换到 HOLD 状态。
          if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

        // 执行安全门停止，进给暂停并禁用主轴/冷却液。
        // 注意：安全门不同于进给暂停，它会无论状态停止一切，禁用电源设备（主轴/冷却液），并阻止恢复直到重新触发。
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
          // 若正在执行 JOG 操作，阻止安全门方法，直到 JOG 取消完成。仅标记此事件。
          if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
            // 检查在恢复停车运动过程中安全门是否重新打开。若已收回、停驻或处于睡眠状态则忽略。
            if (sys.state == STATE_SAFETY_DOOR) {
              if (sys.suspend & SUSPEND_INITIATE_RESTORE) { // 正在恢复
                #ifdef PARKING_ENABLE
                  // 设置暂停并重置适当的控制标志，以重新启动停车序列。
                  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    st_update_plan_block_parameters(); // 通知步进模块重新计算暂停减速。
                    sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                    sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                  } 
                #endif
                sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                sys.suspend |= SUSPEND_RESTART_RETRACT;
              }
            }
            if (sys.state != STATE_SLEEP) { sys.state = STATE_SAFETY_DOOR; }
          }
          // 注意：此标志在门关闭时不会更改，不同于 sys.state。确保任何停车运动在门关闭且状态返回 HOLD 时执行。
          sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
        }
        
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
    if (rt_exec & EXEC_CYCLE_START) {
      // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
      // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
        // Resume door state when parking motion has retracted and door has been closed.
        if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
          if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
            sys.state = STATE_IDLE; // Set to IDLE to immediately resume the cycle.
          } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
            // Flag to re-energize powered components and restore original position, if disabled by SAFETY_DOOR.
            // NOTE: For a safety door to resume, the switch must be closed, as indicated by HOLD state, and
            // the retraction execution is complete, which implies the initial feed hold is not active. To
            // restore normal operation, the restore procedures must be initiated by the following flag. Once,
            // they are complete, it will call CYCLE_START automatically to resume and exit the suspend.
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
        }
        // Cycle start only when IDLE or when a hold is complete and ready to resume.
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; // Set to restore in suspend routine and cycle start after.
          } else {
            // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
            sys.step_control = STEP_CONTROL_NORMAL_OP; // Restore step control to normal operation
            if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; // Break suspend state.
              sys.state = STATE_CYCLE;
              st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
              st_wake_up();
            } else { // Otherwise, do nothing. Set and resume IDLE state.
              sys.suspend = SUSPEND_DISABLE; // Break suspend state.
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by
      // realtime command execution in the main program, ensuring that the planner re-plans safely.
      // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
      // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
      // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
      if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
        // Hold complete. Set to indicate ready to resume.  Remain in HOLD or DOOR states until user
        // has issued a resume command or reset.
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
        // Motion complete. Includes CYCLE/JOG/HOMING states and jog cancel/motion cancel/soft limit events.
        // NOTE: Motion and jog cancel both immediately return to idle after the hold completes.
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   // For jog cancel, flush buffers and sync positions.
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) { // Only occurs when safety door opens during jog.
          sys.suspend &= ~(SUSPEND_JOG_CANCEL);
          sys.suspend |= SUSPEND_HOLD_COMPLETE;
          sys.state = STATE_SAFETY_DOOR;
        } else {
          sys.suspend = SUSPEND_DISABLE;
          sys.state = STATE_IDLE;
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  // Execute overrides.
  rt_exec = sys_rt_exec_motion_override; // Copy volatile sys_rt_exec_motion_override
  if (rt_exec) {
    system_clear_exec_motion_overrides(); // Clear all motion override flags.

    uint8_t new_f_override =  sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min(new_f_override,MAX_FEED_RATE_OVERRIDE);
    new_f_override = max(new_f_override,MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; // Set to report change immediately
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  rt_exec = sys_rt_exec_accessory_override;
  if (rt_exec) {
    system_clear_exec_accessory_overrides(); // Clear all accessory override flags.

    // NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
    uint8_t last_s_override =  sys.spindle_speed_ovr;
    if (rt_exec & EXEC_SPINDLE_OVR_RESET) { last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) { last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) { last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT; }
    last_s_override = min(last_s_override,MAX_SPINDLE_SPEED_OVERRIDE);
    last_s_override = max(last_s_override,MIN_SPINDLE_SPEED_OVERRIDE);

    if (last_s_override != sys.spindle_speed_ovr) {
      bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
      sys.spindle_speed_ovr = last_s_override;
      sys.report_ovr_counter = 0; // Set to report change immediately
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
      // Spindle stop override allowed only while in HOLD state.
      // NOTE: Report counters are set in spindle_set_state() when spindle stop is executed.
      if (sys.state == STATE_HOLD) {
        if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
        else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
      }
    }

    // NOTE: Since coolant state always performs a planner sync whenever it changes, the current
    // run state can be determined by checking the parser state.
    if (rt_exec & (EXEC_COOLANT_FLOOD_OVR_TOGGLE | EXEC_COOLANT_MIST_OVR_TOGGLE)) {
      if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
        uint8_t coolant_state = gc_state.modal.coolant;
        if (rt_exec & EXEC_COOLANT_MIST_OVR_TOGGLE) {
          if (coolant_state & COOLANT_MIST_ENABLE) { bit_false(coolant_state,COOLANT_MIST_ENABLE); }
          else { coolant_state |= COOLANT_MIST_ENABLE; }
        }
        if (rt_exec & EXEC_COOLANT_FLOOD_OVR_TOGGLE) {
          if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
          else { coolant_state |= COOLANT_FLOOD_ENABLE; }
        }
        coolant_set_state(coolant_state); // Report counter set in coolant_set_state().
        gc_state.modal.coolant = coolant_state;
      }
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif

  // Reload step segment buffer
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


// Handles Grbl system suspend procedures, such as feed hold, safety door, and parking motion.
// The system will enter this loop, create local variables for suspend tasks, and return to
// whatever function that invoked the suspend, such that Grbl resumes normal operation.
// This function is written in a way to promote custom parking motions. Simply use this as a
// template
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
    // 声明并初始化停车的局部变量
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
    pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
  #endif

  plan_block_t *block = plan_get_current_block();
  uint8_t restore_condition;
  float restore_spindle_speed;
  if (block == NULL) {
    restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
    restore_spindle_speed = gc_state.spindle_speed;
  } else {
    restore_condition = block->condition;
    restore_spindle_speed = block->spindle_speed;
  }
  #ifdef DISABLE_LASER_DURING_HOLD
    if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) { 
      system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP);
    }
  #endif

  while (sys.suspend) {

    if (sys.abort) { return; }

    // 阻塞，直到初始保持完成并且机器已停止运动。
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

      // 停车管理器。处理去/恢复通电，检查开关状态，以及安全门和休眠状态的停车运动。
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
      
        // Handles retraction motions and de-energizing.
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {

          // 确保在安全门例程开始时禁用所有先前的主轴停止覆盖。
          sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;

          #ifndef PARKING_ENABLE

            spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
            coolant_set_state(COOLANT_DISABLE);     // De-energize

          #else
					
            // Get current position and store restore location and spindle retract waypoint.
            system_convert_array_steps_to_mpos(parking_target,sys_position);
            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              memcpy(restore_target,parking_target,sizeof(parking_target));
              retract_waypoint += restore_target[PARKING_AXIS];
              retract_waypoint = min(retract_waypoint, PARKING_TARGET);
            }

            // Execute slow pull-out parking retract motion. Parking requires homing enabled, the
            // current location not exceeding the parking target location, and laser mode disabled.
            // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
            if ((bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) &&
                            (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                            bit_isfalse(settings.flags, BITFLAG_LASER_MODE)) {

              // 通过拉出距离缩回主轴。确保缩回运动远离工件且路径点运动不超过停车目标位置。
              if (parking_target[PARKING_AXIS] < retract_waypoint) {
                parking_target[PARKING_AXIS] = retract_waypoint;
                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // 保持配件状态
                pl_data->spindle_speed = restore_spindle_speed;
                mc_parking_motion(parking_target, pl_data);
              }

              // NOTE: Clear accessory state after retract and after an aborted restore motion.
              pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
              pl_data->spindle_speed = 0.0;
              spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
              coolant_set_state(COOLANT_DISABLE); // De-energize

              // 执行快速停车缩回运动到停车目标位置。
              if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                parking_target[PARKING_AXIS] = PARKING_TARGET;
                pl_data->feed_rate = PARKING_RATE;
                mc_parking_motion(parking_target, pl_data);
              }

            } else {

              // Parking motion not possible. Just disable the spindle and coolant.
              // NOTE: Laser mode does not start a parking motion to ensure the laser stops immediately.
              spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
              coolant_set_state(COOLANT_DISABLE);     // De-energize

            }

          #endif

          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {

          
          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            // Spindle and coolant should already be stopped, but do it again just to be sure.
            spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
            coolant_set_state(COOLANT_DISABLE); // De-energize
            st_go_idle(); // Disable steppers
            while (!(sys.abort)) { protocol_exec_rt_system(); } // Do nothing until reset.
            return; // Abort received. Return to re-initialize.
          }    
          
          // 允许从停车/安全门恢复。主动检查安全门是否关闭且准备恢复。
          if (sys.state == STATE_SAFETY_DOOR) {
            if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); // Reset door ajar flag to denote ready to resume.
            }
          }

          // 处理停车恢复和安全门恢复。
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              // 执行快速恢复运动到拉出位置。停车需要启用归位。
              // 注意：状态将保持为 DOOR，直到去电和缩回完成。
              if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
                // 检查以确保运动不低于拉出位置。
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

            // 延迟任务：重新启动主轴和冷却液，延迟以供电，然后恢复循环。
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              // 如果安全门在先前的恢复操作期间重新打开，则阻塞。
              if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
                  // 激光模式下忽略主轴启动延迟。设置为在循环启动时打开激光。
                  bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                } else {
                  spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
                  delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                }
              }
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) {
              // 如果安全门在先前的恢复操作期间重新打开，则阻塞。
              if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                // 注意：激光模式将遵循此延迟。排气系统通常由此引脚控制。
                coolant_set_state((restore_condition & (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_FLOOD)));
                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
              }
            }


            #ifdef PARKING_ENABLE
              // 从拉出位置执行缓慢插入运动以恢复到原位置。
              if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
                // 如果在先前的恢复操作期间安全门重新打开，则阻塞。
                if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                  // 无论缩回停车运动是否为有效/安全的运动，恢复停车运动在逻辑上都应是有效的，
                  // 要么通过有效的机器空间返回到原始位置，要么根本不移动。
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
                  pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // 恢复附件状态
                  pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); // 设置恢复程序。
            }
          }

        }

      } else {

        // 进给保持管理器。控制主轴停止覆盖状态。
        // 注意：保持通过在暂停例程开始时的条件检查确保完成。
        if (sys.spindle_stop_ovr) {
          // 处理主轴停止的开始
          if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              spindle_set_state(SPINDLE_DISABLE, 0.0); // 去电
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED; // 设置停止覆盖状态为启用（如果去电）。
            } else {
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; // 清除停止覆盖状态
            }
          // 处理主轴状态的恢复
          } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              report_feedback_message(MESSAGE_SPINDLE_RESTORE);
              if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
                // 激光模式下，忽略主轴启动延迟。设置为在循环开始时打开激光。
                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
              } else {
                spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
              }
            }
            if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
              system_set_exec_state_flag(EXEC_CYCLE_START);  // 设置恢复程序。
            }
            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; // 清除停止覆盖状态
          }
        } else {
          // 在保持期间处理主轴状态。注意：在保持状态下，主轴速度覆盖可能会被更改。
          // 注意：在步进生成器中恢复时，STEP_CONTROL_UPDATE_SPINDLE_PWM 会自动重置。
          if (bit_istrue(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
            spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
            bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
          }
        }

      }
    }
    
    #ifdef SLEEP_ENABLE
      // 检查休眠条件并在超时后执行自动停车。
      // 如果主轴或冷却液已开启或设置为重新启用，休眠对保持和门状态均有效。
      sleep_check();
    #endif

    protocol_exec_rt_system();

  }
}
