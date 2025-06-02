/*
  motion_control.h - 发出运动命令的高级接口
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/
#ifndef motion_control_h
#define motion_control_h

// 系统运动命令的行号必须为零。

#define HOMING_CYCLE_LINE_NUMBER 0
#define PARKING_MOTION_LINE_NUMBER 0

#define HOMING_CYCLE_ALL  0  // 必须为零。
#define HOMING_CYCLE_X    bit(X_AXIS)
#define HOMING_CYCLE_Y    bit(Y_AXIS)
#define HOMING_CYCLE_Z    bit(Z_AXIS)
#define HOMING_CYCLE_A    bit(A_AXIS)

// 在绝对毫米坐标系中执行线性运动。进给速率以毫米/秒为单位，
// 除非 invert_feed_rate 为真。此时，feed_rate 意味着运动应在
// (1 分钟)/feed_rate 的时间内完成。
void mc_line(float *target, plan_line_data_t *pl_data);

// 以偏移模式格式执行弧线。position == 当前 xyz，target == 目标 xyz，
// offset == 当前 xyz 的偏移，axis_XXX 定义工具空间中的圆面，axis_linear 是
// 螺旋运动的方向，radius == 圆半径，is_clockwise_arc 布尔值。用于
// 向量变换方向。
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
  uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc);

// 停留特定的秒数
void mc_dwell(float seconds);

// 执行归位循环以定位机器零点。需要限位开关。
void mc_homing_cycle(uint8_t cycle_mask);

// 执行工具长度探测循环。需要探测开关。
uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags);

// 规划并执行单个特殊的停车运动案例。独立于主规划缓冲区。
void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data);

// 执行系统重置。如果处于运动状态，则停止所有运动并设置系统警报。
void mc_reset();

#endif
