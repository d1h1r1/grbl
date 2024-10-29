/*
  settings.h - EEPROM 配置处理
  Grbl 的一部分

  版权所有 (c) 2011-2016 Sungeun K. Jeon，Gnea Research LLC
  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl 是自由软件：您可以在 GNU 通用公共许可证的条款下重新分发和/或修改
  该许可证由自由软件基金会发布，版本 3，或
  （根据您的选择）任何更高版本。

  Grbl 的发行是希望它能对您有用，
  但不提供任何担保；甚至没有对
  适销性或特定用途的适用性的默示担保。有关更多细节，请参阅
  GNU 通用公共许可证。

  您应该已收到 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请查看 <http://www.gnu.org/licenses/>。
*/

#ifndef settings_h
#define settings_h

#include "grbl.h"

#ifndef EEPROM_LINE_SIZE
  #define EEPROM_LINE_SIZE 80 // EEPROM 行大小
#endif

// EEPROM 数据的版本。将在固件升级时用于从旧版本的 Grbl 迁移现有数据。
// 始终存储在 EEPROM 的字节 0 中
#define SETTINGS_VERSION 10  // 注意：移动到下一个版本时，请检查 settings_reset()。

// 定义 settings.flag 中布尔设置的位标志掩码。
#define BITFLAG_REPORT_INCHES      bit(0) // 报告英寸
#define BITFLAG_LASER_MODE         bit(1) // 激光模式
#define BITFLAG_INVERT_ST_ENABLE   bit(2) // 反转使能
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3) // 硬限制使能
#define BITFLAG_HOMING_ENABLE      bit(4) // 回零使能
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5) // 软限制使能
#define BITFLAG_INVERT_LIMIT_PINS  bit(6) // 反转限位引脚
#define BITFLAG_INVERT_PROBE_PIN   bit(7) // 反转探针引脚

// 在 settings.status_report_mask 中定义状态报告布尔使能位标志
#define BITFLAG_RT_STATUS_POSITION_TYPE     bit(0) // 实时状态位置类型
#define BITFLAG_RT_STATUS_BUFFER_STATE      bit(1) // 实时状态缓冲区状态

// 定义设置恢复位标志。
#define SETTINGS_RESTORE_DEFAULTS bit(0) // 恢复默认设置
#define SETTINGS_RESTORE_PARAMETERS bit(1) // 恢复参数
#define SETTINGS_RESTORE_STARTUP_LINES bit(2) // 恢复启动行
#define SETTINGS_RESTORE_BUILD_INFO bit(3) // 恢复构建信息
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL 0xFF // 所有位标志
#endif

// 定义 Grbl 设置和参数的 EEPROM 存储地址值
#define EEPROM_ADDR_GLOBAL         1U // 全局设置地址
#define EEPROM_ADDR_PARAMETERS     512U // 参数地址
#define EEPROM_ADDR_STARTUP_BLOCK  768U // 启动块地址
#define EEPROM_ADDR_BUILD_INFO     942U // 构建信息地址

// 定义坐标参数的 EEPROM 地址索引
#define N_COORDINATE_SYSTEM 6  // 支持的工作坐标系数量（从索引 1 开始）
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // 存储的系统总数（从索引 0 开始）
// 注意：工作坐标索引为 (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // 家庭位置 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // 家庭位置 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // 坐标偏移 (不支持 G92.2,G92.3)

// 定义 Grbl 轴设置编号方案。从 START_VAL 开始，每次增加 INCREMENT，最多 N_SETTINGS。
#define AXIS_N_SETTINGS          4 // 轴设置数量
#define AXIS_SETTINGS_START_VAL  100 // 注意：保留设置值 >= 100 用于轴设置。最多到 255。
#define AXIS_SETTINGS_INCREMENT  10  // 必须大于轴设置的数量

// 全局持久设置（从字节 EEPROM_ADDR_GLOBAL 开始存储）
typedef struct {
  // 轴设置
  float steps_per_mm[N_AXIS]; // 每毫米步数
  float max_rate[N_AXIS]; // 最大速率
  float acceleration[N_AXIS]; // 加速度
  float max_travel[N_AXIS]; // 最大行程

  // 其余 Grbl 设置
  uint8_t pulse_microseconds; // 脉冲持续时间（微秒）
  uint8_t step_invert_mask; // 步进反转掩码
  uint8_t dir_invert_mask; // 方向反转掩码
  uint8_t stepper_idle_lock_time; // 如果最大值为 255，则步进电机不禁用。
  uint8_t status_report_mask; // 指示所需报告数据的掩码。
  float junction_deviation; // 交汇偏差
  float arc_tolerance; // 弧容差
  
  float rpm_max; // 最大转速
  float rpm_min; // 最小转速
  
  uint8_t flags;  // 包含默认布尔设置

  uint8_t homing_dir_mask; // 回零方向掩码
  float homing_feed_rate; // 回零进给速率
  float homing_seek_rate; // 回零搜索速率
  uint16_t homing_debounce_delay; // 回零去抖延迟
  float homing_pulloff; // 回零拉出距离
} settings_t;
extern settings_t settings; // 全局设置变量

// 初始化配置子系统（从 EEPROM 加载设置）
void settings_init();

// 辅助函数，用于清除和恢复 EEPROM 默认值
void settings_restore(uint8_t restore_flag);

// 从命令行设置新设置的辅助方法
uint8_t settings_store_global_setting(uint8_t parameter, float value);

// 将协议行变量存储为 EEPROM 中的启动行
void settings_store_startup_line(uint8_t n, char *line);

// 读取 EEPROM 启动行到协议行变量
uint8_t settings_read_startup_line(uint8_t n, char *line);

// 存储用户定义的构建信息字符串
void settings_store_build_info(char *line);

// 读取用户定义的构建信息字符串
uint8_t settings_read_build_info(char *line);

// 将选定的坐标数据写入 EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// 从 EEPROM 读取选定的坐标数据
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

// 根据 Grbl 的内部轴编号返回步进引脚掩码
uint8_t get_step_pin_mask(uint8_t i);

// 根据 Grbl 的内部轴编号返回方向引脚掩码
uint8_t get_direction_pin_mask(uint8_t i);

// 根据 Grbl 的内部轴编号返回限位引脚掩码
uint8_t get_limit_pin_mask(uint8_t i);

#endif
