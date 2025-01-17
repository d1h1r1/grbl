/*
  motion_control.c - 发出运动指令的高级接口
  是 Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon, Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以在自由软件基金会发布的 GNU 通用公共许可证的条款下重新发布和/或修改它，
  该许可证的版本可以是许可证的第 3 版，或者（由您选择）任何更高版本。

  Grbl 的发布是希望它能够有用，
  但没有任何保证；甚至没有适销性或特定用途适用性的隐含保证。有关更多详细信息，请参见
  GNU 通用公共许可证。

  您应该已经收到了 GNU 通用公共许可证的副本
  以及 Grbl。如果没有，请访问 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

// 执行绝对毫米坐标的线性运动。给出的进给速率以毫米/秒为单位，
// 除非 invert_feed_rate 为真。然后，feed_rate 意味着运动应该在
// (1 分钟)/feed_rate 的时间内完成。
// 注意：这是 Grbl 规划器的主要入口。所有的直线运动，包括弧线段，
// 都必须通过这个例程，然后再传递给规划器。mc_line 和 plan_buffer_line 的分离
// 主要是为了将非规划器类型的功能与规划器分开，并使得反向间隙补偿或
// 罐装循环集成简单直接。
void mc_line(float *target, plan_line_data_t *pl_data)
{
  // 如果启用，检查软限制违规。将所有线性运动放在这里以从 Grbl 的任何地方捕获。
  if (bit_istrue(settings.flags, BITFLAG_SOFT_LIMIT_ENABLE))
  {
    // 注意：阻止 jog 状态。Jogging 是特殊情况，软限制独立处理。
    if (sys.state != STATE_JOG)
    {
      limits_soft_check(target);
    }
  }

  // 如果处于检查 G-code 模式，阻止运动并阻止规划器。软限制仍然有效。
  if (sys.state == STATE_CHECK_MODE)
  {
    return;
  }

  // 注意：可以在这里安装反向间隙补偿。它需要方向信息来跟踪何时
  // 在预期的线性运动之前插入反向间隙运动，并且需要自己的
  // plan_check_full_buffer() 和检查系统中止循环。同时，对于位置报告，
  // 反向步数也需要进行跟踪，这需要在系统级别进行管理。
  // 可能还有其他一些事情需要跟踪。然而，我们认为
  // 反向间隙补偿不应该由 Grbl 本身处理，因为有许多
  // 方法可以实现它，并且对于不同的 CNC 机器来说可能有效或无效。
  // 这将更好地通过接口作为后处理任务来处理，其中原始 G-code
  // 被翻译并插入最适合机器的反向运动。
  // 注意：也许作为折衷，只需要发送一个标志或特殊命令，
  // 指示 Grbl 这是一个反向间隙补偿运动，这样 Grbl 执行移动但
  // 不更新机器位置值。由于 G-code 解析器和规划器使用的
  // 位置值与系统机器位置是分开的，这是可行的。

  // 如果缓冲区已满：很好！这意味着我们在机器人前面。
  // 在此循环中保持，直到缓冲区中有空间。
  do
  {
    protocol_execute_realtime(); // 检查任何运行时命令
    if (sys.abort)
    {
      return;
    } // 如果系统中止，则退出。
    if (plan_check_full_buffer())
    {
      protocol_auto_cycle_start();
    } // 当缓冲区满时自动循环开始。
    else
    {
      break;
    }
  } while (1);

  // 将运动计划并排入规划器缓冲区
  // uint8_t plan_status; // 在正常操作中未使用。
  plan_buffer_line(target, pl_data);
}

// 执行偏移模式格式的弧线。position == 当前 xyz，target == 目标 xyz，
// offset == 当前 xyz 的偏移，axis_X 定义工具空间中的圆平面，axis_linear 是
// 螺旋移动的方向，radius == 圆半径，isclockwise 布尔值。用于
// 矢量变换方向。
// 该弧线通过生成大量微小的线性段进行近似。每个段的弦容忍度
// 在 settings.arc_tolerance 中配置，该值定义为段到圆的最大法线
// 距离，当端点都位于圆上时。
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
            uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0]; // 从中心到当前位置的半径向量
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  // 从圆心到位置和目标之间的逆时针角度。只需进行一次 atan2() 三角计算。
  float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
  if (is_clockwise_arc)
  { // 根据方向修正 atan2 输出
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON)
    {
      angular_travel -= 2 * M_PI;
    }
  }
  else
  {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON)
    {
      angular_travel += 2 * M_PI;
    }
  }

  // 注意：段的端点位于弧上，这可能导致弧的直径减小，最多可达到
  // (2x) settings.arc_tolerance。对于 99% 的用户，这完全可以。如果需要不同的弧段拟合
  // ，即最小二乘，弧中的中点，只需更改 mm_per_arc_segment 的计算。
  // 对于 Grbl 的预期用途，此值在最严格的情况下不应超过 2000。
  uint16_t segments = floor(fabs(0.5 * angular_travel * radius) /
                            sqrt(settings.arc_tolerance * (2 * radius - settings.arc_tolerance)));

  if (segments)
  {
    // 乘以逆进给速率以补偿此运动由多个离散段近似的事实。逆进给速率应该对所有段的总和是正确的。
    if (pl_data->condition & PL_COND_FLAG_INVERSE_TIME)
    {
      pl_data->feed_rate *= segments;
      bit_false(pl_data->condition, PL_COND_FLAG_INVERSE_TIME); // 强制为绝对进给模式，适用于弧段。
    }

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear]) / segments;

    /* 通过变换矩阵进行向量旋转：r 是原始向量，r_T 是旋转后的向量，
       phi 是旋转角度。解决方案来自 Jens Geisler。
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi)] * r ;

       对于弧生成，圆心是旋转轴，半径向量是
       从圆心到初始位置的定义。每个线段通过连续的
       向量旋转形成。在极少数情况下，单精度值可能会累积大于工具精度的误差。
       因此，实现了精确的弧路径修正。该方法避免了过多非常
       计算昂贵的三角函数操作 [sin()、cos()、tan()]，每次计算可能需要 100-200 微秒。

       小角度近似可以进一步减少计算开销。三阶近似
       （二阶 sin() 的误差太大）适用于大多数，甚至所有 CNC 应用。注意，当 theta_per_segment 大于
       ~0.25 弧度（14 度）且该近似在没有修正的情况下连续使用几十次时，
       该近似将开始累积数值漂移误差。这种情况极不可能发生，因为段长度和 theta_per_segment 是自动生成
       并由弧公差设置缩放的。只有非常大的弧公差设置，对于 CNC
       应用来说是不现实的，才会导致这种数值漂移误差。然而，最好将 N_ARC_CORRECTION 设置在
       低约 4 到高约 20 之间，以避免三角函数操作，同时保持弧生成的准确性。

       该近似还允许 mc_arc 立即将线段插入到规划器中
       而无需计算 cos() 或 sin() 的初始开销。到弧需要应用
       修正时，规划器应该已赶上因初始 mc_arc 开销造成的滞后。
       当存在连续的弧运动时，这一点很重要。
    */
    // 计算：cos_T = 1 - theta_per_segment^2 / 2，sin_T = theta_per_segment - theta_per_segment^3 / 6，在约 52 微秒内
    float cos_T = 2.0 - theta_per_segment * theta_per_segment;
    float sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;

    for (i = 1; i < segments; i++)
    { // 增量（segments - 1）。

      if (count < N_ARC_CORRECTION)
      {
        // 应用向量旋转矩阵。约 40 微秒
        r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
        r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
        r_axis1 = r_axisi;
        count++;
      }
      else
      {
        // 对半径向量进行弧修正。仅每 N_ARC_CORRECTION 次增量计算一次。约 375 微秒
        // 通过将变换矩阵应用于初始半径向量（=-offset）来计算确切位置。
        cos_Ti = cos(i * theta_per_segment);
        sin_Ti = sin(i * theta_per_segment);
        r_axis0 = -offset[axis_0] * cos_Ti + offset[axis_1] * sin_Ti;
        r_axis1 = -offset[axis_0] * sin_Ti - offset[axis_1] * cos_Ti;
        count = 0;
      }

      // 更新弧目标位置
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;

      mc_line(position, pl_data);

      // 在系统中止时中断圆弧。运行时命令检查已由 mc_line 执行。
      if (sys.abort)
      {
        return;
      }
    }
  }
  // 确保最后一段到达目标位置。
  mc_line(target, pl_data);
}

// 执行秒数的停留。
void mc_dwell(float seconds)
{
  if (sys.state == STATE_CHECK_MODE)
  {
    return;
  }
  protocol_buffer_synchronize();
  delay_sec(seconds, DELAY_MODE_DWELL);
}

// 执行归零循环以定位和设置机器零点。只有 '$H' 执行此命令。
// 注意：在执行归零循环之前，缓冲区中不应有运动，并且 Grbl 必须处于空闲状态。
// 这可以防止归零后缓冲计划不正确。
void mc_homing_cycle(uint8_t cycle_mask)
{
// 如果已经启用硬限制，则检查并中止归零循环。这有助于防止
// 硬件限位开关在行程两端都接线到同一个限位引脚时出现问题。
// TODO：将特定引脚的 LIMIT_PIN 调用移到 limits.c 中作为一个函数。
#ifdef LIMITS_TWO_SWITCHES_ON_AXES
  if (limits_get_state())
  {
    mc_reset(); // 发出系统重置命令，确保主轴和冷却液关闭。
    system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
    return;
  }
#endif

  limits_disable(); // 禁用硬限制引脚更改寄存器以便在循环期间使用

  // -------------------------------------------------------------------------------------
  // 执行归零程序。注意：特殊运动情况。只有系统重置有效。

#ifdef HOMING_SINGLE_AXIS_COMMANDS
  if (cycle_mask)
  {
    limits_go_home(cycle_mask);
  } // 根据掩码执行归零循环。
  else
#endif
  {
    // 以更快的归零搜索速度使所有轴的限位开关接通。
    limits_go_home(HOMING_CYCLE_0); // 归零循环 0
#ifdef HOMING_CYCLE_1
    limits_go_home(HOMING_CYCLE_1); // 归零循环 1
#endif
#ifdef HOMING_CYCLE_2
    limits_go_home(HOMING_CYCLE_2); // 归零循环 2
#endif
#ifdef HOMING_CYCLE_3
    limits_go_home(HOMING_CYCLE_3); // 归零循环 3
#endif
#ifdef HOMING_CYCLE_4
    limits_go_home(HOMING_CYCLE_4); // 归零循环 4
#endif
#ifdef HOMING_CYCLE_5
    limits_go_home(HOMING_CYCLE_5); // 归零循环 5
#endif
  }

  protocol_execute_realtime(); // 检查重置并设置系统中止。
  if (sys.abort)
  {
    return;
  } // 未完成。由 mc_alarm 设置的警报状态。

  // 归零循环完成！设置系统以正常运行。
  // -------------------------------------------------------------------------------------

  // 同步 G-code 解析器和规划器位置到归零位置。
  gc_sync_position();
  plan_sync_position();

  // 如果启用了硬限制功能，在归零循环后重新启用硬限制引脚更改寄存器。
  limits_init();
}

// 执行工具长度探测循环。需要探测开关。
// 注意：探测失败后，程序将停止并进入 ALARM 状态。
uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags)
{
  // TODO：需要更新此循环，以遵循非自动循环启动。
  if (sys.state == STATE_CHECK_MODE)
  {
    return (GC_PROBE_CHECK_MODE);
  }

  // 在开始探测循环之前，完成所有排队命令并清空规划器缓冲区。
  protocol_buffer_synchronize();
  if (sys.abort)
  {
    return (GC_PROBE_ABORT);
  } // 如果已经发出系统重置，则返回。

  // 初始化探测控制变量
  uint8_t is_probe_away = bit_istrue(parser_flags, GC_PARSER_PROBE_IS_AWAY);
  uint8_t is_no_error = bit_istrue(parser_flags, GC_PARSER_PROBE_IS_NO_ERROR);
  sys.probe_succeeded = false; // 在开始循环之前重新初始化探测历史。
  probe_configure_invert_mask(is_probe_away);

  // 在同步后，检查探测是否已触发。如果已触发，则停止并发出警报。
  // 注意：此探测初始化错误适用于所有探测循环。
  if (probe_get_state())
  { // 检查探测引脚状态。
    system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_INITIAL);
    protocol_execute_realtime();
    probe_configure_invert_mask(false); // 返回之前重新初始化反转掩码。
    return (GC_PROBE_FAIL_INIT);        // 除了退出别无选择。
  }

  // 设置并排队探测运动。自动循环启动不应开始循环。
  mc_line(target, pl_data);

  // 在步进模块中激活探测状态监控。
  sys_probe_state = PROBE_ACTIVE;

  // 执行探测循环。这里等待直到探测触发或运动完成。
  system_set_exec_state_flag(EXEC_CYCLE_START);
  do
  {
    protocol_execute_realtime();
    if (sys.abort)
    {
      return (GC_PROBE_ABORT);
    } // 检查系统是否中止
  } while (sys.state != STATE_IDLE);

  // 探测循环完成！

  // 如果探测失败且启用了带错误的循环，则设置状态变量并输出错误。
  if (sys_probe_state == PROBE_ACTIVE)
  {
    if (is_no_error)
    {
      memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    }
    else
    {
      system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_CONTACT);
    }
  }
  else
  {
    sys.probe_succeeded = true; // 表示探测循环成功完成。
  }
  sys_probe_state = PROBE_OFF;        // 确保探测状态监控被禁用。
  probe_configure_invert_mask(false); // 重新初始化反转掩码。
  protocol_execute_realtime();        // 检查并执行实时命令

  // 重置步进器和规划器缓冲区，以清除探测运动的剩余部分。
  st_reset();           // 重置步进段缓冲区。
  plan_reset();         // 重置规划器缓冲区。将规划器位置归零。确保探测运动被清除。
  plan_sync_position(); // 将规划器位置同步到当前机器位置。

#ifdef MESSAGE_PROBE_COORDINATES
                        // 所有完成！输出探测位置作为消息。
  report_probe_parameters();
#endif

  if (sys.probe_succeeded)
  {
    return (GC_PROBE_FOUND);
  } // 成功的探测循环。
  else
  {
    return (GC_PROBE_FAIL_END);
  } // 在运动范围内未能触发探测。无论是否有错误。
}

// 规划并执行停车的单一特殊运动情况。独立于主规划器缓冲区。
// 注意：使用始终免费的规划器环形缓冲区头存储运动参数以供执行。
void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data)
{
  if (sys.abort)
  {
    return;
  } // 在中止时阻塞。

  uint8_t plan_status = plan_buffer_line(parking_target, pl_data);

  if (plan_status)
  {
    bit_true(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
    bit_false(sys.step_control, STEP_CONTROL_END_MOTION); // 如果进给保持处于活动状态，允许执行停车运动。
    st_parking_setup_buffer();                            // 为特殊停车运动情况设置步进段缓冲区
    st_prep_buffer();
    st_wake_up();
    do
    {
      protocol_exec_rt_system();
      if (sys.abort)
      {
        return;
      }
    } while (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION);
    st_parking_restore_buffer(); // 恢复步进段缓冲区到正常运行状态。
  }
  else
  {
    bit_false(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
    protocol_exec_rt_system();
  }
}

// 准备系统重置的方法，通过设置实时重置命令并终止任何
// 在系统中活动的进程。这也检查在 Grbl
// 处于运动状态时是否发出了系统重置。如果是，终止步进器并将系统警报设置为标记位置
// 丢失，因为发生了突然的非控制减速。由
// 实时中止命令和硬限制在中断级别调用。因此，尽量保持到最低限度。
void mc_reset()
{
  // 只有这个函数可以设置系统重置。帮助防止多次终止调用。
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET))
  {
    system_set_exec_state_flag(EXEC_RESET);

    // 停止主轴和冷却液。
    spindle_stop();
    coolant_stop();

    // 仅在处于任何运动状态（即循环、主动保持或归零）时终止步进器。
    // 注意：如果通过步进空闲延迟设置保持步进器启用，则这也通过完全避免
    // go_idle 调用来保持步进器启用，除非运动状态被破坏，否则一切都无效。
    if ((sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG)) ||
        (sys.step_control & (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION)))
    {
      if (sys.state == STATE_HOMING)
      {
        if (!sys_rt_exec_alarm)
        {
          system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET);
        }
      }
      else
      {
        system_set_exec_alarm(EXEC_ALARM_ABORT_CYCLE);
      }
      st_go_idle(); // 强制终止步进器。位置可能已经丢失。
    }
  }
}
