#include "button_status_read.h"
#include "main.h"

/* 
 * 统一读取船型开关物理档位。
 * 
 * 【语法与架构说明】
 * 1. 返回类型 (lock_switch_pos_t)：这里是“枚举(enum)”类型。
 *    - 为什么不用 int？枚举能实现“防呆”，明确限制返回值只能是 high/low，不仅在 IDE 里会有代码补全提示，更能被编译器检查，避免别人乱传数字（如 return 5;）。
 * 2. 隔离防腐层 (三目运算符 ?:)：
 *    - 把底层的 HAL_GPIO_ReadPin() 和硬件宏 (GPIO_PIN_SET) 拦截在底层，
 *    - 转换成我们自己定义的业务层枚举 (lock_switch_pos_high)。以后换芯片不影响上层逻辑。
 * 3. static：修饰函数表示“内部函数”，不暴露到文件外，类似于面向对象的 private。
 * 4. (void)：严格表示该函数不仅没有参数，也“绝对不接受参数”，防止胡乱传参。
 */
static lock_switch_pos_t lock_switch_read_pin(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET)
               ? lock_switch_pos_high
               : lock_switch_pos_low;
}

/**
 * @brief 锁定船型开关初始化
 * @note 记录系统上电时的开关初始档位
 */
void lock_switch_init(lock_switch_ctx_t *ctx)
{
    /* 防御式编程：ctx == 0 意为检查指针是否为空(NULL)，防止后续解引用导致系统HardFault奔溃 */
    if (ctx == 0) {
        return;
    }

    ctx->init_pos = lock_switch_read_pin();
    ctx->current_pos = ctx->init_pos;
    
    /* 1U 中的 U 表示 Unsigned，与 uint8_t 类型匹配，属于嵌入式规范写法，避免编译器有符号数强转警告 */
    ctx->initialized = 1U;
    ctx->first_unlocked = 0U;
    
    /* 高级连击变量初始化 */
    ctx->last_toggle_tick = 0U;
    ctx->toggle_count = 0U;
    ctx->is_super_mode_locked = 0U;
}

void lock_switch_update(lock_switch_ctx_t *ctx)
{
    /* 作为局部变量存储"上一拍"和"这一拍"的状态，因为它们只在本次比较中用到，不用塞进结构体占用长效内存。
       这里使用枚举类型(lock_switch_pos_t)代替int可以增加语义明确性并在编译期获得更严格的数值约束 */
    lock_switch_pos_t prev;
    lock_switch_pos_t now;

    if (ctx == 0) {
        return;
    }

    if (ctx->initialized == 0U) {
        lock_switch_init(ctx);
    }

    prev = ctx->current_pos;
    now = lock_switch_read_pin();

    /* 【功能0】硬件防抖（Debounce）：
       机械开关在拨动的瞬间，金属簧片会产生高频颤动（通常在5ms-20ms内），
       如果不加屏蔽，单次拨动会被单片机极其敏锐地捕捉成几十次翻转，直接触发大招。
       这里的逻辑是：如果距离上一次确认的有效翻转还没过 50ms (死区时间)，直接无视当前的电平变化。 */
    uint32_t current_tick = HAL_GetTick(); // 获取系统毫秒级时间戳
    if (now != prev && (current_tick - ctx->last_toggle_tick > 50U)) {
        
        /* 确实是有效翻转，更新当前保存的档位 */
        ctx->current_pos = now;

        /* 【功能1】开机防误触：只要拨动过一次，就永久解除最初的开机限制 */
        if ((ctx->first_unlocked == 0U) && (now != ctx->init_pos)) {
            ctx->first_unlocked = 1U;
        }

        /* 【功能2】暴击连招：判断距离上一次波拨动的时间是否在狭窄窗口内（比如 800ms 算一次有效连续拨动） */
        if (current_tick - ctx->last_toggle_tick <= 800U) {
            ctx->toggle_count++;
        } else {
            /* 如果你拨一下发呆了1秒再拨，或者这是开机第一次拨，重置计作第1下 */
            ctx->toggle_count = 1U;
        }
        
        ctx->last_toggle_tick = current_tick; // 刷新时间戳给下一次防抖和连击用

        /* 【功能3】永久死锁模式：连续拨动达到3下以后，触发条件 */
        if (ctx->toggle_count >= 3U) {
            ctx->is_super_mode_locked = 1U; /* 这个标志位置1后如果不加干预，将一直保持直到系统断电 */
        }

        /* 统一抛出事件勾子：
           当检测到电平确实发生有效翻转时，调用这个 __weak 声明的函数。
           这里的精髓是：本层（Middleware Layer）只负责识别硬件和维持状态，不负责具体业务（如亮灯、蜂鸣器响）。
           其他模块可以在自己的 .c 文件里重写同名函数，一旦开关拨动，就会自动执行那个重写的函数。*/
        lock_switch_on_changed(prev, now);
    }
}

lock_switch_pos_t lock_switch_get_current(const lock_switch_ctx_t *ctx)
{
    if (ctx == 0) {
        return lock_switch_pos_low;
    }

    return ctx->current_pos;
}

/* 我们把状态机的实际内存放到文件级静态全局变量里，
   方便下面三个给上层FSM使用的接口随时去读取它的最新状态。 */
static lock_switch_ctx_t g_switch_ctx = {0};

int fsm_enable(void)
{
    /* 在主循环中反复跑，喂狗更新状态 */
    lock_switch_update(&g_switch_ctx);

    return (g_switch_ctx.first_unlocked != 0U) ? 1 : 0;
}

int lock_switch_is_machine_locked(void)
{
    /* 常规机器加解锁（比如FSM通过这个来决定能不能发力）。
       逻辑：假设物理上往上打(High)代表物理 Lock锁住，往下打(Low)代表Free释放 */
    return (g_switch_ctx.current_pos == lock_switch_pos_high) ? 1 : 0;
}

int lock_switch_is_super_mode(void)
{
    /* 向外界报告：兄弟有没有触发三连击大招！ */
    return (g_switch_ctx.is_super_mode_locked != 0U) ? 1 : 0;
}

__weak void lock_switch_on_changed(lock_switch_pos_t from, lock_switch_pos_t to)
{
    /* 默认空实现：由业务模块按需重写。 */
    (void)from;
    (void)to;
}