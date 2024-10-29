/*
  protocol.c - 控制Grbl执行协议和程序
  Grbl的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改
  它，许可证的版本为3，或（根据您的选择）任何更高版本。

  Grbl的分发是希望它会有用，
  但不提供任何担保；甚至没有对适销性或特定用途适用性的暗示担保。有关详细信息，请参见
  GNU通用公共许可证。

  您应该已经收到了GNU通用公共许可证的副本
  随Grbl一起。如果没有，请参见<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 定义行标志。包括注释类型跟踪和行溢出检测。
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)

static char line[LINE_BUFFER_SIZE]; // 要执行的行。以零终止。

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
  // 在重置、错误或初始启动时检查并报告警报状态。
  // 注意：睡眠模式禁用步进电机驱动程序，无法保证位置。
  // 重新初始化睡眠状态为警报模式，以确保用户归位或确认。
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // 确保警报状态被设置。
  } else {
    // 检查安全门是否打开。
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // 进入安全门模式。应返回为IDLE状态。
    }
    // 一切正常！
    system_execute_startup(line); // 执行启动脚本。
  }

  // ---------------------------------------------------------------------------------
  // 主循环！在系统中止时，返回到main()以重置系统。
  // 这是Grbl在等待任务时处于空闲状态的地方。
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {
    // 处理一行传入的串行数据，当数据可用时进行处理。执行初步过滤，去除空格和注释并将所有字母大写。
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // 行结束

        protocol_execute_realtime(); // 运行时命令检查点。
        if (sys.abort) { return; } // 系统中止时返回调用函数

        line[char_counter] = 0; // 设置字符串终止字符。
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // 直接执行一行格式化输入，并报告执行状态。
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // 报告行溢出错误。
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // 空行或注释行。用于同步目的。
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl '$' 系统命令
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // 其他所有为G代码。如果处于警报或点动模式则阻塞。
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // 解析并执行G代码块。
          report_status_message(gc_execute_line(line));
        }

        // 重置下一行的跟踪数据。
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // 丢弃所有（结束符外）注释字符和溢出字符。
          if (c == ')') {
            // 结束'()'注释。恢复允许的行。
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // 丢弃空白和控制字符
          } else if (c == '/') {
            // 不支持块删除。忽略该字符。
            // 注意：如果支持，仅需检查系统是否启用块删除。
          } else if (c == '(') {
            // 启用注释标志，并忽略所有字符，直到')'或行结束。
            // 注意：这并不完全遵循NIST定义，但现在足够用了。
            // 将来，我们可以简单地删除注释内的项目，但保留注释控制字符，以便G代码解析器可以进行错误检查。
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // 注意：';'到行结束的注释是LinuxCNC定义，而非NIST。
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: 安装'%'特性
          // } else if (c == '%') {
            // 程序开始-结束百分号不支持。
            // 注意：这可能被安装以告诉Grbl程序何时正在运行与手动输入，
            // 在程序运行期间，系统自动循环开始将继续执行
            // 直到下一个'%'符号。这将帮助修复某些
            // 函数在执行其任务时清空计划缓冲区的恢复问题。
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // 检测到行缓冲区溢出并设置标志。
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // 将小写字母转换为大写
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    // 如果串行读取缓冲区中没有更多字符需要处理和执行， 
    // 这表明G代码流可能已填满计划缓冲区或已完成。
    // 在任一情况下，如果启用自动循环开始，则排队移动。
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // 运行时命令检查点。
    if (sys.abort) { return; } // 系统中止时返回main()程序循环以重置系统。

    #ifdef SLEEP_ENABLE
      // 检查睡眠条件，如果超时，执行自动停靠。
      sleep_check();    
    #endif
  }

  return; /* 永远不会到达 */
}



// 阻塞直到所有缓冲的步进被执行完毕或处于循环状态。在同步调用期间可与暂停保持配合工作，
// 还会等待干净的循环结束。
void protocol_buffer_synchronize()
{
  // 如果系统处于排队状态，确保在自动启动标志存在时循环恢复。
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // 检查并执行实时命令
    if (sys.abort) { return; } // 检查系统是否中止
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}

// 自动循环开始触发条件为有运动准备执行且主程序未在
// 主动解析命令。
// 注意：此函数仅从主循环、缓冲同步和 mc_line() 调用，并在
// 下列条件之一存在时执行：没有更多块被发送（即流传输结束，单个命令），
// 需要等待缓冲中的运动执行的命令调用缓冲同步，
// 或规划器缓冲区已满且准备好执行。
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // 检查缓冲区是否有任何块。
    system_set_exec_state_flag(EXEC_CYCLE_START); // 如果有，就执行它们！
  }
}

// 此函数是 Grbl 实时命令执行系统的通用接口。它被从主程序中的各种检查点调用，
// 主要在可能等待缓冲区清理空间的 while 循环或最后检查点的执行时间可能超过一秒的点。
// 这是以异步方式（即多任务）执行实时命令与 Grbl 的 G-code 解析和规划功能的一种方式。
// 此函数还作为中断的接口，用于设置系统实时标志，
// 只有主程序处理这些标志，消除了定义更多计算开销大的易失性变量的需要。
// 这也提供了一种控制方式来执行某些任务，而不会产生两个或多个相同任务的实例，
// 比如在进给保持或覆盖时重新计算缓冲区。
// 注意：sys_rt_exec_state 变量标志由任何过程、步进或串行中断、引脚输出、限位开关或主程序设置。
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}

// 在需要时执行实时命令。此函数主要作为 Grbl 的状态机，
// 控制 Grbl 提供的各种实时功能。
// 注意：除非您确切知道自己在做什么，否则请不要更改此内容！
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // 临时变量以避免多次调用易失性变量。
  rt_exec = sys_rt_exec_alarm; // 复制易失性 sys_rt_exec_alarm。
  if (rt_exec) { // 仅在任何位标志为真时进入
    // 系统警报。由于某些严重错误，所有操作已关闭。向用户报告
    // 错误源。如果严重，Grbl 通过进入无限循环禁用系统，直到系统重置/中止。
    sys.state = STATE_ALARM; // 设置系统警报状态
    report_alarm_message(rt_exec);
    // 在关键事件标志下停止所有操作。目前硬限制和软限制会标记此。
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // 禁用任何现有重置
      do {
        // 阻塞所有操作，除重置和状态报告外，直到用户发出重置或电源
        // 循环。硬限制通常发生在无人值守或未关注时。给
        // 用户和 GUI 时间做必要的操作再重置，例如终止
        // 输入流。软限制也同样适用。虽然位置不会丢失，但继续流传输可能导致严重崩溃。
      } while (bit_isfalse(sys_rt_exec_state, EXEC_RESET));
    }
    system_clear_exec_alarm(); // 清除警报
  }

  rt_exec = sys_rt_exec_state; // 复制易失性 sys_rt_exec_state。
  if (rt_exec) {
    // 执行系统中止。
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // 仅在此处设置为真。
      return; // 无需其他操作，直接退出。
    }

    // 执行并串行打印状态
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    // 注意：一旦启动暂停，系统会立即进入暂停状态，以阻止所有
    // 主程序过程，直到重置或恢复。这确保了暂停安全完成。
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

      // 检查暂停方法的允许状态。
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
        // 如果处于循环或慢速状态，立即启动运动暂停。
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { // 如果已经暂停，则阻止。
            st_update_plan_block_parameters(); // 通知步进模块重新计算暂停减速。
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; // 启动暂停状态并激活标志。
            if (sys.state == STATE_JOG) { // 任何暂停事件下取消慢速。
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        // 如果处于空闲状态，Grbl 并未运动。只需指示暂停状态并完成暂停。
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        // 执行并标记运动取消，带减速并返回到空闲状态。主要用于探测循环
        // 中止并取消其余运动。
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // 运动取消仅发生在循环中，但可能在之前发起了暂停和安全门
          // 以保持循环。运动取消仅对单个规划块运动有效，而慢速取消
          // 将处理并清除多个规划块运动。
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // 注意：状态为 STATE_CYCLE。
        }

        // 如果需要，则执行减速的进给保持。然后，暂停系统。
        if (rt_exec & EXEC_FEED_HOLD) {
          // 阻止安全门、慢速和休眠状态改变为保持状态。
          if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

        // 执行带减速的安全门停止，并禁用主轴/冷却液。
        // 注意：安全门与进给保持的区别在于无论状态如何都停止所有操作，禁用供电
        // 设备（主轴/冷却液），并在开关重新接通之前阻止恢复。
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
          // 如果在慢速模式下，则在慢速取消完成之前阻止安全门操作。仅标记已发生。
          if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
            // 仅在恢复停车运动期间检查安全是否重新打开。已忽略正在收回、停车或
            // 处于睡眠状态的情况。
            if (sys.state == STATE_SAFETY_DOOR) {
              if (sys.suspend & SUSPEND_INITIATE_RESTORE) { // 正在恢复
                #ifdef PARKING_ENABLE
                  // 设置保持并重置适当的控制标志以重新启动停车序列。
                  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    st_update_plan_block_parameters(); // 通知步进模块重新计算暂停减速。
                    sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                    sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                  } // 否则 NO_MOTION 处于活动状态。
                #endif
                sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                sys.suspend |= SUSPEND_RESTART_RETRACT;
              }
            }
            if (sys.state != STATE_SLEEP) { sys.state = STATE_SAFETY_DOOR; }
          }
          // 注意：当门关闭时，此标志不会改变，与 sys.state 不同。确保在门开关关闭
          // 时执行任何停车运动，状态返回到保持状态。
          sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
        }
        
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE | SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    // 通过启动步进中断来执行循环开始，以开始执行队列中的块。
    if (rt_exec & EXEC_CYCLE_START) {
      // 如果在同一时间调用了暂停命令：进给保持、运动取消和安全门，则阻止。
      // 确保自动循环开始不会在没有用户输入的情况下恢复暂停。
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
        // 当停车运动已收回并且门已关闭时，恢复门状态。
        if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
          if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
            sys.state = STATE_IDLE; // 设置为 IDLE 以立即恢复循环。
          } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
            // 标记以重新通电并恢复原始位置，如果被安全门禁用。
            // 注意：为了恢复安全门，开关必须关闭，如 HOLD 状态所示，且
            // 收回执行完成，这意味着初始进给保持未激活。为了
            // 恢复正常操作，恢复程序必须由以下标志发起。一旦完成，
            // 将自动调用 CYCLE_START 以恢复并退出暂停。
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
        }
        // 循环仅在空闲或保持完成并准备恢复时开始。
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; // 设置以在暂停例程中恢复，并在之后启动循环。
          } else {
            // 仅在规划器缓冲区中存在排队运动并且未取消运动的情况下启动循环。
            sys.step_control = STEP_CONTROL_NORMAL_OP; // 恢复步进控制为正常操作
            if (plan_get_current_block() && bit_isfalse(sys.suspend, SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; // 打破暂停状态。
              sys.state = STATE_CYCLE;
              st_prep_buffer(); // 在开始循环之前初始化步骤段缓冲。
              st_wake_up();
            } else { // 否则，不执行任何操作。设置并恢复到 IDLE 状态。
              sys.suspend = SUSPEND_DISABLE; // 打破暂停状态。
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      // 在进给保持之后重新初始化循环计划和步进系统，以便恢复。由
      // 主程序中的实时命令执行调用，确保规划器安全地重新规划。
      // 注意：Bresenham 算法变量在规划器和步进器
      // 循环重新初始化中仍然保持。步进路径应如同没有发生任何事情一样继续。
      // 注意：EXEC_CYCLE_STOP 由步进子系统在循环或进给保持完成时设置。
      if ((sys.state & (STATE_HOLD | STATE_SAFETY_DOOR | STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
        // 暂停完成。设置以指示准备恢复。保持在 HOLD 或 DOOR 状态，直到用户
        // 发出恢复命令或重置。
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control, (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
        // 运动完成。包括 CYCLE/JOG/HOMING 状态和慢速取消/运动取消/软限制事件。
        // 注意：运动和慢速取消在保持完成后立即返回空闲状态。
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   // 对于慢速取消，清空缓冲区并同步位置。
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) { // 仅在慢速时安全门打开时发生。
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

  // 执行覆盖。
  rt_exec = sys_rt_exec_motion_override; // 复制易失性 sys_rt_exec_motion_override
  if (rt_exec) {
    system_clear_exec_motion_overrides(); // 清除所有运动覆盖标志。

    uint8_t new_f_override = sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min(new_f_override, MAX_FEED_RATE_OVERRIDE);
    new_f_override = max(new_f_override, MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; // 设置为立即报告变化
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

rt_exec = sys_rt_exec_accessory_override;
if (rt_exec) {
    system_clear_exec_accessory_overrides(); // 清除所有附件覆盖标志。

    // 注意：与运动覆盖不同，主轴覆盖不需要重新初始化规划器。
    uint8_t last_s_override = sys.spindle_speed_ovr;
    if (rt_exec & EXEC_SPINDLE_OVR_RESET) { last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) { last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) { last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT; }
    last_s_override = min(last_s_override, MAX_SPINDLE_SPEED_OVERRIDE);
    last_s_override = max(last_s_override, MIN_SPINDLE_SPEED_OVERRIDE);

    if (last_s_override != sys.spindle_speed_ovr) {
        bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
        sys.spindle_speed_ovr = last_s_override;
        sys.report_ovr_counter = 0; // 设置为立即报告变化
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
        // 仅在 HOLD 状态下允许主轴停止覆盖。
        // 注意：主轴停止时的报告计数器在 spindle_set_state() 中设置。
        if (sys.state == STATE_HOLD) {
            if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
            else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
        }
    }

    // 注意：由于冷却液状态在每次更改时始终执行规划器同步，因此可以通过检查解析器状态来确定当前运行状态。
    if (rt_exec & (EXEC_COOLANT_FLOOD_OVR_TOGGLE | EXEC_COOLANT_MIST_OVR_TOGGLE)) {
        if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
            uint8_t coolant_state = gc_state.modal.coolant;
            if (rt_exec & EXEC_COOLANT_MIST_OVR_TOGGLE) {
                if (coolant_state & COOLANT_MIST_ENABLE) { bit_false(coolant_state, COOLANT_MIST_ENABLE); }
                else { coolant_state |= COOLANT_MIST_ENABLE; }
            }
            if (rt_exec & EXEC_COOLANT_FLOOD_OVR_TOGGLE) {
                if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state, COOLANT_FLOOD_ENABLE); }
                else { coolant_state |= COOLANT_FLOOD_ENABLE; }
            }
            coolant_set_state(coolant_state); // 在 coolant_set_state() 中设置报告计数器。
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

// 重新加载步进段缓冲区
if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP | STATE_JOG)) {
    st_prep_buffer();
}


// 处理 Grbl 系统的暂停程序，如进给保持、安全门和停放运动。
// 系统将进入此循环，为暂停任务创建局部变量，并返回到
// 调用暂停的任何函数，以便 Grbl 恢复正常操作。
// 此函数的编写方式旨在促进自定义停放动作。只需将其用作
// 模板
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
    // 声明并初始化停放局部变量
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

    // 阻塞，直到初始保持完成，机器停止运动。
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

      // 停放管理器。处理去/再通电、开关状态检查以及
      // 安全门和睡眠状态的停放运动。
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
      
        // 处理收回动作和去通电。
        if (bit_isfalse(sys.suspend, SUSPEND_RETRACT_COMPLETE)) {

          // 确保在安全门例程开始时禁用任何先前的主轴停止覆盖。
          sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;

          #ifndef PARKING_ENABLE

            spindle_set_state(SPINDLE_DISABLE, 0.0); // 去通电
            coolant_set_state(COOLANT_DISABLE);     // 去通电

          #else
					
            // 获取当前位置并存储恢复位置和主轴收回航点。
            system_convert_array_steps_to_mpos(parking_target, sys_position);
            if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
              memcpy(restore_target, parking_target, sizeof(parking_target));
              retract_waypoint += restore_target[PARKING_AXIS];
              retract_waypoint = min(retract_waypoint, PARKING_TARGET);
            }

            // 执行慢速拉出停放收回运动。停放要求启用归位，
            // 当前地点不超过停放目标位置，并且激光模式禁用。
            // 注意：状态将在去通电和收回完成之前保持为 DOOR。
            if ((bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) &&
                            (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                            bit_isfalse(settings.flags, BITFLAG_LASER_MODE)) {

              // 按拉出距离收回主轴。确保收回运动远离
              // 工件，并且航点运动不超过停放目标位置。
              if (parking_target[PARKING_AXIS] < retract_waypoint) {
                parking_target[PARKING_AXIS] = retract_waypoint;
                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // 保留附加状态
                pl_data->spindle_speed = restore_spindle_speed;
                mc_parking_motion(parking_target, pl_data);
              }

              // 注意：在收回后和中止恢复运动后清除附加状态。
              pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
              pl_data->spindle_speed = 0.0;
              spindle_set_state(SPINDLE_DISABLE, 0.0); // 去通电
              coolant_set_state(COOLANT_DISABLE); // 去通电

              // 执行快速停放收回运动到停放目标位置。
              if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                parking_target[PARKING_AXIS] = PARKING_TARGET;
                pl_data->feed_rate = PARKING_RATE;
                mc_parking_motion(parking_target, pl_data);
              }

            } else {

              // 停放运动不可用。只需禁用主轴和冷却液。
              // 注意：激光模式不会启动停放运动，以确保激光立即停止。
              spindle_set_state(SPINDLE_DISABLE, 0.0); // 去通电
              coolant_set_state(COOLANT_DISABLE);     // 去通电

            }

          #endif

          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {

          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            // 主轴和冷却液应该已经停止，但为了确保再做一次。
            spindle_set_state(SPINDLE_DISABLE, 0.0); // 去通电
            coolant_set_state(COOLANT_DISABLE); // 去通电
            st_go_idle(); // 禁用步进电机
            while (!(sys.abort)) { protocol_exec_rt_system(); } // 在重置之前不执行任何操作。
            return; // 收到中止。返回重新初始化。
          }    
          
          // 允许从停放/安全门恢复。主动检查安全门是否关闭并准备恢复。
          if (sys.state == STATE_SAFETY_DOOR) {
            if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); // 重置门未关标志以表示准备恢复。
            }
          }

          // 处理停放恢复和安全门恢复。
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              // 执行快速恢复运动到拉出位置。停放要求启用归位。
              // 注意：状态将在去通电和收回完成之前保持为 DOOR。
              if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
                // 检查以确保运动不低于拉出位置。
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

            // 延迟任务：重新启动主轴和冷却液，延迟上电，然后恢复循环。
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              // 如果在先前的恢复操作期间安全门重新打开，则阻塞。
              if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
                  // 在激光模式下，忽略主轴加速延迟。设置为在循环开始时开启激光。
                  bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                } else {
                  spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
                  delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                }
              }
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) {
              // 如果在先前的恢复操作期间安全门重新打开，则阻塞。
              if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                // 注意：激光模式将遵循此延迟。排气系统通常由此引脚控制。
                coolant_set_state((restore_condition & (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_FLOOD)));
                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
              }
            }

            #ifdef PARKING_ENABLE
              // 执行从拉出位置到恢复位置的慢速下降运动。
              if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
                // 如果在先前的恢复操作期间安全门重新打开，则阻塞。
                if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
                  // 不论收回停放运动是否是有效/安全运动，
                  // 恢复停放运动应在逻辑上是有效的，要么通过返回到
                  // 原始位置通过有效机器空间，要么根本不移动。
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
                  pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // 恢复附加状态
                  pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend, SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); // 设置为恢复程序。
            }
          }

        }

      } else {

        // 进给保持管理器。控制主轴停止覆盖状态。
        // 注意：在暂停例程开始时，通过条件检查确保保持完成。
        if (sys.spindle_stop_ovr) {
          // 处理主轴停止的开始
          if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              spindle_set_state(SPINDLE_DISABLE, 0.0); // 去通电
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED; // 设置停止覆盖状态为启用，如果去通电。
            } else {
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; // 清除停止覆盖状态
            }
          // 处理主轴状态的恢复
          } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              report_feedback_message(MESSAGE_SPINDLE_RESTORE);
              if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
                // 在激光模式下，忽略主轴加速延迟。设置为在循环开始时开启激光。
                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
              } else {
                spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
              }
            }
            if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
              system_set_exec_state_flag(EXEC_CYCLE_START);  // 设置为恢复程序。
            }
            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; // 清除停止覆盖状态
          }
        } else {
          // 处理保持期间的主轴状态。注意：主轴速度覆盖可能会在保持状态下被更改。
          // 注意：在步进发生器中，STEP_CONTROL_UPDATE_SPINDLE_PWM会在恢复时自动重置。
          if (bit_istrue(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
            spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
            bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
          }
        }

      }
    }
    
    #ifdef 0
      // 检查睡眠条件并执行自动停放，如果超时。
      // 在保持和门状态下，若主轴或冷却液处于开启状态或
      // 被设置为重新启用，睡眠是有效的。
      sleep_check();
    #endif

    protocol_exec_rt_system();

  }
}
