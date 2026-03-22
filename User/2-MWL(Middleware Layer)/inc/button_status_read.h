#ifndef __BUTTON_STATUS_READ_H
#define __BUTTON_STATUS_READ_H

#include <stdint.h>

typedef enum {
    lock_switch_pos_low = 0,
    lock_switch_pos_high = 1,
} lock_switch_pos_t;

typedef struct {

    lock_switch_pos_t init_pos;
    /* 最近一次采样到的档位 */
    lock_switch_pos_t current_pos;
    /* 上下文是否已完成初始化 */
    uint8_t initialized;
    /* 是否完成了开机首次防误触解锁（替代原来的 changed_once） */
    uint8_t first_unlocked;
    /* ================= 高级连招状态 ================= */
    /* 记录上次拨动的系统时间(ms)，用来实现"连续快速拨动"判定 */
    uint32_t last_toggle_tick;
    /* 短期内连续拨动的次数 */
    uint8_t toggle_count;
    /* 连续拨动3次产生的"永久特殊运动模式"标志位，一旦置1只能断电恢复 */
    uint8_t is_super_mode_locked;
} lock_switch_ctx_t;

/*
 * 读取当前船型开关并记录为初始档位。
 * 通常在第一次调用 update 时自动完成，不需要手动调用。
 */
void lock_switch_init(lock_switch_ctx_t *ctx);

/*
 * 周期调用：持续采样开关状态。
 * 一旦发现当前档位和上电初始档位不同，会把 changed_once 置位。
 */
void lock_switch_update(lock_switch_ctx_t *ctx);

/*
 * 系统启动门控接口：
 * 返回 0 表示还未换档，系统保持等待；
 * 返回 1 表示已发生首次换档，系统允许进入工作态。
 */
int fsm_enable(void);

/* 获取常规解/锁机状态：如 1=锁定机体(Lock)，0=正常工作(Free) */
int lock_switch_is_machine_locked(void);

/* 获取是否触发了连续3次拨动的"永久电机维持模式" */
int lock_switch_is_super_mode(void);

/* 获取最近一次采样到的档位。 */
lock_switch_pos_t lock_switch_get_current(const lock_switch_ctx_t *ctx);

/*
 * 换档事件回调（弱定义）：
 * 可在其他模块重写，用于后续扩展功能（告警、日志、模式切换等）。
 */
void lock_switch_on_changed(lock_switch_pos_t from, lock_switch_pos_t to);



#endif
