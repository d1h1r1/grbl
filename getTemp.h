/*
  jog.h - 移动方法
  Grbl 的一部分

  版权所有 (c) 2016 Sungeun K. Jeon，Gnea Research LLC

  Grbl 是自由软件：你可以在自由软件基金会发布的 GNU 通用公共许可证条款下重新分发和/或修改
  它，许可证版本为 3，或（根据你的选择）任何更高版本。

  Grbl 的发布是为了希望它能有用，
  但不提供任何担保；甚至没有关于
  适销性或适用于特定目的的隐含担保。有关详细信息，请参见
  GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

#ifndef getTemp_h
#define getTemp_h


// 系统运动行号必须为零。
#define JOG_LINE_NUMBER 0

// 设置从 G-code 解析器接收到的有效移动命令，检查软限位，并执行移动。
float ds18b20_read_temp();

#endif
