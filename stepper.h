/*
  stepper.h - 步进电机驱动：使用步进电机执行 planner.c 的运动计划
  Grbl的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon, Gnea Research LLC
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

#ifndef stepper_h
#define stepper_h

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 10
#endif

// 初始化和设置步进电机子系统
void stepper_init();

// 启用步进电机，但除非由运动控制或实时命令调用，否则不会启动循环。
void st_wake_up();

// 立即禁用步进电机
void st_go_idle();

// 生成步进和方向端口的反转掩码。
void st_generate_step_dir_invert_masks();

// 重置步进子系统变量
void st_reset();

// 更改步进段缓冲区的运行状态以执行特殊停车运动。
void st_parking_setup_buffer();

// 在停车运动后将步进段缓冲区恢复到正常运行状态。
void st_parking_restore_buffer();

// 重新加载步进段缓冲区。由实时执行系统持续调用。
void st_prep_buffer();

// 当正在执行的块被新计划更新时由 planner_recalculate() 调用。
void st_update_plan_block_parameters();

// 如果在 config.h 中启用了实时速率报告，则由实时状态报告调用。
float st_get_realtime_rate();

#endif
