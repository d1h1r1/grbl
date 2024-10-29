/*
  sleep.h - 睡眠方法头文件
  Grbl的一部分
  
  版权所有 (c) 2016 Sungeun K. Jeon  

  Grbl 是自由软件：您可以根据 GNU 通用公共许可证的条款重新分发和/或修改
  它，该许可证由自由软件基金会发布，许可证的版本为第 3 版，或
  （根据您的选择）任何更高版本。

  Grbl 的发行目的是希望它对您有用，
  但不提供任何担保；甚至没有对适销性或特定目的适用性的暗示担保。有关更多详细信息，请参阅
  GNU 通用公共许可证。

  您应该已经收到了一份 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#ifndef sleep_h
#define sleep_h

#include "grbl.h"

// 初始化睡眠计时器
void sleep_init();

// 检查睡眠的运行条件。如果满足条件，则启用睡眠倒计时并在经过后执行
// 睡眠模式。
void sleep_check();

#endif
