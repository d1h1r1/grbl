/*
  protocol.h - 控制 Grbl 执行协议和程序
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：你可以根据自由软件基金会发布的 GNU 通用公共许可证的条款重新分发和/或修改它，版本为许可证的第 3 版，或（根据你的选择）任何更高版本。

  Grbl 以希望它会有用的方式发布，但不提供任何担保；甚至不包括对适销性或特定用途适用性的隐含担保。有关更多详细信息，请参阅 GNU 通用公共许可证。

  你应该已经收到一份 GNU 通用公共许可证的副本，随 Grbl 一起。如果没有，请参阅 <http://www.gnu.org/licenses/>。
*/

#ifndef protocol_h
#define protocol_h

// 从串行输入流到要执行的行缓冲区大小。
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 256
#endif

// 启动 Grbl 主循环。它处理来自串行端口的所有输入字符，并在完成时执行它们。
// 它还负责完成初始化过程。
void protocol_main_loop();

// 在主程序的各个停止点检查并执行实时命令
void protocol_execute_realtime();
void protocol_exec_rt_system();

// 执行自动循环功能（如果启用）。
void protocol_auto_cycle_start();

// 阻塞直到所有缓冲步骤执行完毕
void protocol_buffer_synchronize();

#endif
