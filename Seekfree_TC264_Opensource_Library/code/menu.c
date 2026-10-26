/*********************************************************************************************************************
* 文件名称：menu.c
* 功能说明：二级菜单，一级菜单选页面，二级页面分别是图像、参数、电机PWM、数据、发车
* 页面内容：一级菜单列表 / 二值化图像 / P、D、环岛PD、陀螺仪、曝光度、差速参数、电机PWM / 图像数据 / 发车倒计时
* 保存说明：参数只保存在RAM中，掉电后恢复默认值
* 按键说明：
*           运行页面  S4 = P22_2：进入菜单
*           一级菜单  S1 = P32_4：移动光标
*                     S3 = P23_1：确定键，进入光标选中的页面（选到 4 START 即发车）
*           二级页面  S4 = P22_2：返回一级菜单（调参页还可用 S1切换参数、S2减小、S3增大）
*           发车页面  一级菜单选到 4 START 按 S3 后，先停顿 MENU_START_DELAY_MS 毫秒再切到运行页面
********************************************************************************************************************/

#include "menu.h"
#include "control.h"
#include "PID.h"
#include "Island.h"

// P参数最小值
#define MENU_KP_MIN              (0.00f)

// P参数最大值
#define MENU_KP_MAX              (8.0f)

// P参数调整步长
#define MENU_KP_STEP             (0.01f)

// kgyro参数最小值
#define MENU_KGYRO_MIN           (0.00f)

// kgyro参数最大值
#define MENU_KGYRO_MAX           (2.00f)

// kgyro参数调整步长
#define MENU_KGYRO_STEP          (0.01f)

// D参数最小值
#define MENU_KD_MIN              (0.00f)

// D参数最大值
#define MENU_KD_MAX              (6.0f)

// D参数调整步长
#define MENU_KD_STEP             (0.01f)

/* 差速参数菜单范围：
 * Diff Gain ：0.00~1.50，步长0.02；
 * Diff Limit：0~1500 PWM，步长50；
 * Diff Rate ：0~500 PWM/帧，步长10。 */

// 差速增益系数最小值
#define MENU_DIFF_GAIN_MIN       (0.00f)

// 差速增益系数最大值
#define MENU_DIFF_GAIN_MAX       (15.0f)

// 差速增益系数调整步长
#define MENU_DIFF_GAIN_STEP      (0.1f)

// 差速绝对限幅最小值
#define MENU_DIFF_MAX_MIN        (0)

// 差速绝对限幅最大值
#define MENU_DIFF_MAX_MAX        (1600)

// 差速绝对限幅调整步长
#define MENU_DIFF_MAX_STEP       (50)

// 差速每帧最大变化量最小值
#define MENU_DIFF_SLEW_MIN       (0)

// 差速每帧最大变化量最大值
#define MENU_DIFF_SLEW_MAX       (1000)

// 差速每帧最大变化量调整步长
#define MENU_DIFF_SLEW_STEP      (10)

// 摄像头曝光度最小值
#define MENU_EXPOSURE_MIN        (0)

// 摄像头曝光度最大值，太大可能降低帧率
#define MENU_EXPOSURE_MAX        (1000)

// 摄像头曝光度调整步长
#define MENU_EXPOSURE_STEP       (5)

// 摄像头帧率最小值
#define MENU_FPS_MIN             (20)

// 摄像头帧率最大值
#define MENU_FPS_MAX             (60)

// 摄像头帧率调整步长
#define MENU_FPS_STEP            (5)

// 参数页一次显示的行数
#define MENU_PARAM_ROWS          (5)

// 电机PWM最小值
#define MENU_MOTOR_MIN           (0)

// 电机PWM最大值
#define MENU_MOTOR_MAX           (5000)

// 电机PWM调整步长
#define MENU_MOTOR_STEP          (50)

// 二级页刷新分频：每 N 帧刷一次，太重会把按键扫描拖慢
#define MENU_REFRESH_DIV         (5)

// 发车后必须停顿的时间（裁判要求发车后停顿1~2秒），单位毫秒
#define MENU_START_DELAY_MS      (2000)

// 当前选中的参数编号
typedef enum
{
    // 当前选中P
    MENU_ITEM_P = 0,

    // 当前选中D
    MENU_ITEM_D,

    MENU_ITEM_ISLAND_P,

    MENU_ITEM_ISLAND_D,

    // 当前选中Kgyro
    MENU_ITEM_KGYRO,

    // 当前选中摄像头曝光度
    MENU_ITEM_EXPOSURE,

    // 当前选中摄像头帧率
    MENU_ITEM_FPS,

    // 当前选中差速增益系数
    MENU_ITEM_DIFF_GAIN,

    // 当前选中差速绝对限幅
    MENU_ITEM_DIFF_MAX,

    // 当前选中差速每帧最大变化量
    MENU_ITEM_DIFF_SLEW,

    // 当前选中电机PWM
    MENU_ITEM_MOTOR,

    // 当前选中左环岛PWM
    MENU_ITEM_LEFT_ISLAND,

    // 当前选中右环岛PWM
    MENU_ITEM_RIGHT_ISLAND,

    // 当前选中高速直道PWM
    MENU_ITEM_FAST,

    // 参数总数
    MENU_ITEM_COUNT,
} menu_item_enum;

// 当前P参数
static volatile float menu_kp = 1.56f;//1.55

// 当前D参数
static volatile float menu_kd = 0.84f;//0.82，0.84

// 当前Island_P参数
static volatile float menu_island_p = 1.94f;

// 当前Island_D参数
static volatile float menu_island_d = 3.10f;

// 当前kgyro参数
static volatile float menu_kgyro = 0.10f;

// 当前摄像头曝光度：CPU0菜单修改，CPU1读取后调用 mt9v03x_set_exposure_time()
volatile int menu_exposure = 215;

// 当前摄像头目标帧率：CPU0菜单修改，CPU1读取后重新配置摄像头
volatile int menu_fps = 50;

/* 差速增益系数 Diff Gain：
 * 0.00 = 完全关闭差速。 */
static volatile float menu_diff_gain = 0;

/* 差速绝对限幅 Diff Limit：
 * 单位是电机PWM，表示左右轮速度差最多给到多少；
 * 默认600，越小越不容易甩尾，但转向辅助越弱。 */
static volatile int menu_diff_max = 600;//750

/* 差速变化率限制 Diff Rate：
 * 单位是PWM/帧，表示每帧差速最多变化多少；
 * 默认100，越小变化越慢、越平顺；
 * 0表示不做变化率限制，差速可以瞬间变化。 */
static volatile int menu_diff_slew = 100;//200

// 当前电机基础PWM
volatile int menu_motor_pwm = 2550;//2600

// 当前左环岛PWM
volatile int menu_left_island_pwm = 1750;

// 当前右环岛PWM
volatile int menu_right_island_pwm = 1900;

// 当前高速直道PWM
volatile int menu_fast_pwm = 2850;//2850

// 当前选中的参数
static uint8 menu_item = MENU_ITEM_P;

// 1表示菜单页面，0表示图像页面
uint8 menu_page = 0;

// 1表示需要重绘菜单
static uint8 menu_dirty = 1;

// 当前二级页面编号，取值见上面的 MENU_SUB_xxx
uint8 menu_sub = MENU_SUB_LIST;

// 发车标志：只有发车之后 cpu0_main 才会驱动电机
uint8 race_started = 0;

// 一级菜单光标位置 0=图像 1=调参 2=数据
static uint8 menu_cursor = 0;
// 二级页刷新计数
static uint8 menu_refresh_cnt = 0;

// IMAGE页面显示模式：0=二值化图，1=灰度图
static uint8 menu_image_mode = 0;

#define MENU_PROFILE_SLOTS               (8)
#define MENU_PROFILE_PAGE_A              (10)
#define MENU_PROFILE_PAGE_B              (11)
#define MENU_PROFILE_PAGE_NONE           (0xFF)
#define MENU_PROFILE_MAGIC               (0x50524631UL)
#define MENU_PROFILE_VERSION             (2)

#define MENU_PROFILE_NOTICE_NONE         (0)
#define MENU_PROFILE_NOTICE_LOADED       (1)
#define MENU_PROFILE_NOTICE_SAVED        (2)
#define MENU_PROFILE_NOTICE_DEFAULT      (3)
#define MENU_PROFILE_NOTICE_ERROR        (4)

typedef struct
{
    float kp;
    float kd;
    float island_kp;
    float island_kd;
    float kgyro;
    float diff_gain;
    int exposure;
    int diff_max;
    int diff_slew;
    int base_pwm;
    int left_island_pwm;
    int right_island_pwm;
    int fast_pwm;
    int fps;
}menu_profile_params_t;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 slot_count;
    uint32 sequence;
    uint32 active_slot;
    menu_profile_params_t profiles[MENU_PROFILE_SLOTS];
    uint32 crc32;
}menu_profile_store_t;

typedef enum
{
    MENU_PROFILE_ITEM_SLOT = 0,
    MENU_PROFILE_ITEM_LOAD,
    MENU_PROFILE_ITEM_SAVE,
    MENU_PROFILE_ITEM_DEFAULT,
    MENU_PROFILE_ITEM_COUNT,
}menu_profile_item_enum;

static const menu_profile_params_t menu_profile_defaults =
{
    1.56f,
    0.84f,
    1.94f,
    3.10f,
    0.10f,
    0.0f,
    215,
    600,
    100,
    2550,
    1750,
    1900,
    2850,
    50,
};

static menu_profile_store_t menu_profiles;
static uint8 menu_profile_active = 0;
static uint8 menu_profile_selected = 0;
static uint8 menu_profile_cursor = 0;
static uint8 menu_profile_current_page = MENU_PROFILE_PAGE_NONE;
static uint8 menu_profile_dirty = 0;
static uint8 menu_profile_notice = MENU_PROFILE_NOTICE_NONE;

static void Menu_DrawProfileTag(uint16 x, uint16 y);

static uint32 Menu_ProfileCrc32(const uint8 *data, uint32 length)
{
    uint32 crc = 0xFFFFFFFFUL;
    uint32 i;
    uint8 bit;

    for(i = 0; i < length; i++)
    {
        crc ^= data[i];
        for(bit = 0; bit < 8; bit++)
        {
            if(crc & 0x00000001UL)
                crc = (crc >> 1) ^ 0xEDB88320UL;
            else
                crc >>= 1;
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static void Menu_ProfileFromRuntime(menu_profile_params_t *profile)
{
    profile->kp = menu_kp;
    profile->kd = menu_kd;
    profile->island_kp = menu_island_p;
    profile->island_kd = menu_island_d;
    profile->kgyro = menu_kgyro;
    profile->diff_gain = menu_diff_gain;
    profile->exposure = menu_exposure;
    profile->diff_max = menu_diff_max;
    profile->diff_slew = menu_diff_slew;
    profile->base_pwm = menu_motor_pwm;
    profile->left_island_pwm = menu_left_island_pwm;
    profile->right_island_pwm = menu_right_island_pwm;
    profile->fast_pwm = menu_fast_pwm;
    profile->fps = menu_fps;
}

static void Menu_ProfileToRuntime(const menu_profile_params_t *profile)
{
    menu_kp = profile->kp;
    menu_kd = profile->kd;
    menu_island_p = profile->island_kp;
    menu_island_d = profile->island_kd;
    menu_kgyro = profile->kgyro;
    menu_diff_gain = profile->diff_gain;
    menu_exposure = profile->exposure;
    menu_diff_max = profile->diff_max;
    menu_diff_slew = profile->diff_slew;
    menu_motor_pwm = profile->base_pwm;
    menu_left_island_pwm = profile->left_island_pwm;
    menu_right_island_pwm = profile->right_island_pwm;
    menu_fast_pwm = profile->fast_pwm;
    menu_fps = profile->fps;
}

static uint8 Menu_ProfileValidate(const menu_profile_store_t *profile_store)
{
    uint32 crc;

    if((profile_store->magic != MENU_PROFILE_MAGIC) ||
       (profile_store->version != MENU_PROFILE_VERSION) ||
       (profile_store->slot_count != MENU_PROFILE_SLOTS) ||
       (profile_store->active_slot >= MENU_PROFILE_SLOTS))
    {
        return 0;
    }

    crc = Menu_ProfileCrc32((const uint8 *)profile_store,
                            (uint32)(sizeof(menu_profile_store_t) - sizeof(uint32)));
    return (crc == profile_store->crc32) ? 1 : 0;
}

static uint8 Menu_ProfileReadPage(uint8 page, menu_profile_store_t *profile_store)
{
    flash_read_page(0, page, (uint32 *)profile_store,
                    (uint16)(sizeof(menu_profile_store_t) / sizeof(uint32)));
    return Menu_ProfileValidate(profile_store);
}

static uint8 Menu_ProfileWriteStore(void)
{
    uint8 target_page;
    uint32 interrupt_state;
    menu_profile_store_t verify_store;

    menu_profiles.magic = MENU_PROFILE_MAGIC;
    menu_profiles.version = MENU_PROFILE_VERSION;
    menu_profiles.slot_count = MENU_PROFILE_SLOTS;
    menu_profiles.sequence++;
    menu_profiles.active_slot = menu_profile_active;
    menu_profiles.crc32 = Menu_ProfileCrc32(
        (const uint8 *)&menu_profiles,
        (uint32)(sizeof(menu_profile_store_t) - sizeof(uint32)));

    if(menu_profile_current_page == MENU_PROFILE_PAGE_A)
        target_page = MENU_PROFILE_PAGE_B;
    else
        target_page = MENU_PROFILE_PAGE_A;

    interrupt_state = interrupt_global_disable();
    flash_erase_page(0, target_page);
    flash_write_page(0, target_page, (const uint32 *)&menu_profiles,
                     (uint16)(sizeof(menu_profile_store_t) / sizeof(uint32)));
    interrupt_global_enable(interrupt_state);

    if(Menu_ProfileReadPage(target_page, &verify_store) &&
       (verify_store.sequence == menu_profiles.sequence))
    {
        menu_profile_current_page = target_page;
        menu_profile_dirty = 0;
        menu_profile_notice = MENU_PROFILE_NOTICE_SAVED;
        menu_dirty = 1;
        return 1;
    }

    menu_profile_notice = MENU_PROFILE_NOTICE_ERROR;
    return 0;
}

static uint8 Menu_ProfileSaveActive(void)
{
    Menu_ProfileFromRuntime(&menu_profiles.profiles[menu_profile_active]);
    return Menu_ProfileWriteStore();
}

static uint8 Menu_ProfileLoadSlot(uint8 slot)
{
    uint8 result;

    if(slot >= MENU_PROFILE_SLOTS)
        return 0;

    if((slot == menu_profile_active) && (!menu_profile_dirty))
    {
        menu_profile_selected = slot;
        menu_profile_notice = MENU_PROFILE_NOTICE_LOADED;
        return 1;
    }

    if(menu_profile_dirty)
        Menu_ProfileFromRuntime(&menu_profiles.profiles[menu_profile_active]);

    menu_profile_active = slot;
    menu_profile_selected = slot;
    Menu_ProfileToRuntime(&menu_profiles.profiles[menu_profile_active]);
    menu_profile_dirty = 0;
    menu_profile_notice = MENU_PROFILE_NOTICE_LOADED;

    result = Menu_ProfileWriteStore();
    menu_profile_notice = result ? MENU_PROFILE_NOTICE_LOADED :
                                   MENU_PROFILE_NOTICE_ERROR;
    return result;
}

static uint8 Menu_ProfileDefaultSlot(uint8 slot)
{
    uint8 result;

    if(slot >= MENU_PROFILE_SLOTS)
        return 0;

    menu_profiles.profiles[slot] = menu_profile_defaults;
    menu_profile_selected = slot;

    if(slot == menu_profile_active)
        Menu_ProfileToRuntime(&menu_profiles.profiles[slot]);

    menu_profile_dirty = 0;
    result = Menu_ProfileWriteStore();
    menu_profile_notice = result ? MENU_PROFILE_NOTICE_DEFAULT :
                                   MENU_PROFILE_NOTICE_ERROR;
    return result;
}

static void Menu_ProfileInit(void)
{
    uint8 i;
    uint8 valid_a;
    uint8 valid_b;
    menu_profile_store_t temp_store;

    menu_profile_current_page = MENU_PROFILE_PAGE_NONE;
    memset(&menu_profiles, 0, sizeof(menu_profiles));
    menu_profiles.magic = MENU_PROFILE_MAGIC;
    menu_profiles.version = MENU_PROFILE_VERSION;
    menu_profiles.slot_count = MENU_PROFILE_SLOTS;
    menu_profiles.sequence = 0;
    menu_profiles.active_slot = 0;

    for(i = 0; i < MENU_PROFILE_SLOTS; i++)
        menu_profiles.profiles[i] = menu_profile_defaults;

    valid_a = Menu_ProfileReadPage(MENU_PROFILE_PAGE_A, &temp_store);

    if(valid_a)
    {
        menu_profiles = temp_store;
        menu_profile_current_page = MENU_PROFILE_PAGE_A;
    }

    valid_b = Menu_ProfileReadPage(MENU_PROFILE_PAGE_B, &temp_store);
    if(valid_b)
    {
        if((!valid_a) ||
           ((int32)(temp_store.sequence - menu_profiles.sequence) > 0))
        {
            menu_profiles = temp_store;
            menu_profile_current_page = MENU_PROFILE_PAGE_B;
        }
    }

    if(menu_profile_current_page == MENU_PROFILE_PAGE_NONE)
    {
        menu_profile_active = 0;
        menu_profile_selected = 0;
    }
    else
    {
        menu_profile_active = (uint8)menu_profiles.active_slot;
        if(menu_profile_active >= MENU_PROFILE_SLOTS)
            menu_profile_active = 0;
        menu_profile_selected = menu_profile_active;
        Menu_ProfileToRuntime(&menu_profiles.profiles[menu_profile_active]);
    }

    menu_profile_dirty = 0;
    menu_profile_notice = MENU_PROFILE_NOTICE_NONE;
}

static void Menu_ProfileMarkDirty(void)
{
    menu_profile_dirty = 1;
}

static uint8 Menu_ProfileSaveDirty(void)
{
    if(!menu_profile_dirty)
        return 1;
    return Menu_ProfileSaveActive();
}

/*********************************************************************************************************************
* 函数名称：Menu_KeyEvent
* 功能说明：查询某个按键有没有事件（直接用库里的按键驱动）
* 参数说明：key 按键编号 KEY_1~KEY_4
* 返回说明：1表示有按键事件，0表示无按键事件
* 备    注：key_init(1) + 1ms定时器里调 key_scanner()，10次采样=10ms，所以点一下就能认
********************************************************************************************************************/
static uint8 Menu_KeyEvent(key_index_enum key)
{
    key_state_enum state = key_get_state(key);

    if((state == KEY_SHORT_PRESS) || (state == KEY_LONG_PRESS))
    {
        key_clear_state(key);
        return 1;
    }

    return 0;
}
/*********************************************************************************************************************
* 函数名称：Menu_Adjust
* 功能说明：调整当前选中的参数
* 参数说明：direction 1表示增大，-1表示减小
* 返回说明：无
********************************************************************************************************************/
static void Menu_Adjust(int direction)
{
    menu_profile_params_t profile_before;
    menu_profile_params_t profile_after;

    Menu_ProfileFromRuntime(&profile_before);

    // 调整P
    if(menu_item == MENU_ITEM_P)
    {
        menu_kp += direction * MENU_KP_STEP;

        if(menu_kp < MENU_KP_MIN)
            menu_kp = MENU_KP_MIN;
        if(menu_kp > MENU_KP_MAX)
            menu_kp = MENU_KP_MAX;
    }
    // 调整D
    else if(menu_item == MENU_ITEM_D)
    {
        menu_kd += direction * MENU_KD_STEP;

        if(menu_kd < MENU_KD_MIN)
            menu_kd = MENU_KD_MIN;
        if(menu_kd > MENU_KD_MAX)
            menu_kd = MENU_KD_MAX;
    }
    // 调整Island_P
    else if(menu_item == MENU_ITEM_ISLAND_P)
    {
        menu_island_p += direction * MENU_KP_STEP;

        if(menu_island_p < MENU_KP_MIN)
            menu_island_p = MENU_KP_MIN;
        if(menu_island_p > MENU_KP_MAX)
            menu_island_p = MENU_KP_MAX;
    }
    // 调整Island_D
    else if(menu_item == MENU_ITEM_ISLAND_D)
    {
        menu_island_d += direction * MENU_KD_STEP;

        if(menu_island_d < MENU_KD_MIN)
            menu_island_d = MENU_KD_MIN;
        if(menu_island_d > MENU_KD_MAX)
            menu_island_d = MENU_KD_MAX;
    }
    // 调整Kgyro
    else if(menu_item == MENU_ITEM_KGYRO)
    {
        menu_kgyro += direction * MENU_KGYRO_STEP;

        if(menu_kgyro < MENU_KGYRO_MIN)
            menu_kgyro = MENU_KGYRO_MIN;
        if(menu_kgyro > MENU_KGYRO_MAX)
            menu_kgyro = MENU_KGYRO_MAX;
    }
    // 调整摄像头曝光度
    else if(menu_item == MENU_ITEM_EXPOSURE)
    {
        menu_exposure += direction * MENU_EXPOSURE_STEP;

        if(menu_exposure < MENU_EXPOSURE_MIN)
            menu_exposure = MENU_EXPOSURE_MIN;
        if(menu_exposure > MENU_EXPOSURE_MAX)
            menu_exposure = MENU_EXPOSURE_MAX;
    }
    // 调整摄像头帧率
    else if(menu_item == MENU_ITEM_FPS)
    {
        menu_fps += direction * MENU_FPS_STEP;

        if(menu_fps < MENU_FPS_MIN)
            menu_fps = MENU_FPS_MIN;
        if(menu_fps > MENU_FPS_MAX)
            menu_fps = MENU_FPS_MAX;
    }
    // 调整差速增益系数
    else if(menu_item == MENU_ITEM_DIFF_GAIN)
    {
        menu_diff_gain += direction * MENU_DIFF_GAIN_STEP;

        if(menu_diff_gain < MENU_DIFF_GAIN_MIN)
            menu_diff_gain = MENU_DIFF_GAIN_MIN;
        if(menu_diff_gain > MENU_DIFF_GAIN_MAX)
            menu_diff_gain = MENU_DIFF_GAIN_MAX;
    }
    // 调整差速绝对限幅
    else if(menu_item == MENU_ITEM_DIFF_MAX)
    {
        menu_diff_max += direction * MENU_DIFF_MAX_STEP;

        if(menu_diff_max < MENU_DIFF_MAX_MIN)
            menu_diff_max = MENU_DIFF_MAX_MIN;
        if(menu_diff_max > MENU_DIFF_MAX_MAX)
            menu_diff_max = MENU_DIFF_MAX_MAX;
    }
    // 调整差速每帧最大变化量
    else if(menu_item == MENU_ITEM_DIFF_SLEW)
    {
        menu_diff_slew += direction * MENU_DIFF_SLEW_STEP;

        if(menu_diff_slew < MENU_DIFF_SLEW_MIN)
            menu_diff_slew = MENU_DIFF_SLEW_MIN;
        if(menu_diff_slew > MENU_DIFF_SLEW_MAX)
            menu_diff_slew = MENU_DIFF_SLEW_MAX;
    }
    // 调整电机PWM
    else if(menu_item == MENU_ITEM_MOTOR)
    {
        menu_motor_pwm += direction * MENU_MOTOR_STEP;

        if(menu_motor_pwm < MENU_MOTOR_MIN)
            menu_motor_pwm = MENU_MOTOR_MIN;
        if(menu_motor_pwm > MENU_MOTOR_MAX)
            menu_motor_pwm = MENU_MOTOR_MAX;
    }
    // 调整左环岛PWM
    else if(menu_item == MENU_ITEM_LEFT_ISLAND)
    {
        menu_left_island_pwm += direction * MENU_MOTOR_STEP;

        if(menu_left_island_pwm < MENU_MOTOR_MIN)
            menu_left_island_pwm = MENU_MOTOR_MIN;
        if(menu_left_island_pwm > MENU_MOTOR_MAX)
            menu_left_island_pwm = MENU_MOTOR_MAX;
    }
    // 调整右环岛PWM
    else if(menu_item == MENU_ITEM_RIGHT_ISLAND)
    {
        menu_right_island_pwm += direction * MENU_MOTOR_STEP;

        if(menu_right_island_pwm < MENU_MOTOR_MIN)
            menu_right_island_pwm = MENU_MOTOR_MIN;
        if(menu_right_island_pwm > MENU_MOTOR_MAX)
            menu_right_island_pwm = MENU_MOTOR_MAX;
    }
    // 调整高速直道PWM
    else if(menu_item == MENU_ITEM_FAST)
    {
        menu_fast_pwm += direction * MENU_MOTOR_STEP;

        if(menu_fast_pwm < MENU_MOTOR_MIN)
            menu_fast_pwm = MENU_MOTOR_MIN;
        if(menu_fast_pwm > MENU_MOTOR_MAX)
            menu_fast_pwm = MENU_MOTOR_MAX;
    }

    // 只有参数实际发生变化时才标记为待保存，限位处继续按键不会触发写Flash
    Menu_ProfileFromRuntime(&profile_after);
    if(memcmp(&profile_before, &profile_after, sizeof(menu_profile_params_t)) != 0)
    {
        Menu_ProfileMarkDirty();
    }

    // 参数变化后需要重绘
    menu_dirty = 1;
}

/*********************************************************************************************************************
* 函数名称：Menu_DrawParamRow
* 功能说明：在参数页的指定行画一个参数
* 参数说明：item 参数编号，y 屏幕纵坐标
* 返回说明：无
********************************************************************************************************************/
static void Menu_DrawParamRow(uint8 item, uint16 y)
{
    // 当前选中的参数用黄色显示
    tft180_set_color((menu_item == item) ? RGB565_YELLOW : RGB565_WHITE,
                     RGB565_BLACK);

    if(item == MENU_ITEM_P)
    {
        tft180_show_string(0, y, "Kp:");
        tft180_show_float(88, y, menu_kp, 4, 2);
    }
    else if(item == MENU_ITEM_D)
    {
        tft180_show_string(0, y, "Kd:");
        tft180_show_float(88, y, menu_kd, 4, 2);
    }
    else if(item == MENU_ITEM_ISLAND_P)
    {
        tft180_show_string(0, y, "Island P:");
        tft180_show_float(88, y, menu_island_p, 4, 2);
    }
    else if(item == MENU_ITEM_ISLAND_D)
    {
        tft180_show_string(0, y, "Island D:");
        tft180_show_float(88, y, menu_island_d, 4, 2);
    }
    else if(item == MENU_ITEM_KGYRO)
    {
        tft180_show_string(0, y, "Gyro K:");
        tft180_show_float(88, y, menu_kgyro, 4, 2);
    }
    else if(item == MENU_ITEM_EXPOSURE)
    {
        tft180_show_string(0, y, "Exposure:");
        tft180_show_int(88, y, menu_exposure, 4);
    }
    else if(item == MENU_ITEM_FPS)
    {
        tft180_show_string(0, y, "FPS:");
        tft180_show_int(88, y, menu_fps, 2);
    }
    else if(item == MENU_ITEM_DIFF_GAIN)
    {
        tft180_show_string(0, y, "Diff Gain:");
        tft180_show_float(88, y, menu_diff_gain, 4, 2);
    }
    else if(item == MENU_ITEM_DIFF_MAX)
    {
        tft180_show_string(0, y, "Diff Limit:");
        tft180_show_int(96, y, menu_diff_max, 4);
    }
    else if(item == MENU_ITEM_DIFF_SLEW)
    {
        tft180_show_string(0, y, "Diff Rate:");
        tft180_show_int(88, y, menu_diff_slew, 4);
    }
}
/*********************************************************************************************************************
* 函数名称：Menu_DrawParam
* 功能说明：绘制参数调整页。参数比较多，所以做成5行滚动窗口
* 参数说明：无
* 返回说明：无
********************************************************************************************************************/
static void Menu_DrawParam(void)
{
    // 参数页总共有 Kp/Kd/Island P/Island D/Gyro K/Exposure/Diff Gain/Diff Limit/Diff Rate 九项
    uint8 item_count = (uint8)(MENU_ITEM_MOTOR - MENU_ITEM_P);
    uint8 selected = (uint8)(menu_item - MENU_ITEM_P);
    uint8 top = 0;
    uint8 row;
    uint8 item;
    uint16 y;

    // 滚动窗口：让当前选中的参数尽量居中，同时不超过参数总数
    if(item_count > MENU_PARAM_ROWS)
    {
        if(selected > (MENU_PARAM_ROWS / 2))
            top = selected - (MENU_PARAM_ROWS / 2);
        if(top > (item_count - MENU_PARAM_ROWS))
            top = item_count - MENU_PARAM_ROWS;
    }

    // 清屏
    tft180_full(RGB565_BLACK);

    // 标题
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    tft180_show_string(0, 0, "PARAMETERS");
    Menu_DrawProfileTag(88, 0);
    tft180_show_int(112, 0, (int32)(selected + 1), 2);
    tft180_show_string(128, 0, "/10");

    // 逐行画窗口里的参数
    for(row = 0; row < MENU_PARAM_ROWS; row++)
    {
        item = (uint8)(top + row);
        if(item >= item_count)
            break;

        y = (uint16)(20 + row * 20);
        Menu_DrawParamRow(item, y);
    }

    // 按键提示
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(0, 116, "S1:NEXT S2- S3+");
}
/*********************************************************************************************************************
* 函数名称：Menu_DrawMotor
* 功能说明：绘制电机PWM调整页
* 参数说明：无
* 返回说明：无
********************************************************************************************************************/
static void Menu_DrawMotor(void)
{
    // 清屏
    tft180_full(RGB565_BLACK);

    // 8x16字体显示参数
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    tft180_show_string(0, 0, "MOTOR PWM");
    Menu_DrawProfileTag(96, 0);

    // 显示基础PWM
    tft180_set_color((menu_item == MENU_ITEM_MOTOR) ? RGB565_YELLOW : RGB565_WHITE,
                     RGB565_BLACK);
    tft180_show_string(0, 24, "Base PWM:");
    tft180_show_int(80, 24, menu_motor_pwm, 4);

    // 显示左环岛PWM
    tft180_set_color((menu_item == MENU_ITEM_LEFT_ISLAND) ? RGB565_YELLOW : RGB565_WHITE,
                     RGB565_BLACK);
    tft180_show_string(0, 46, "Left Island:");
    tft180_show_int(104, 46, menu_left_island_pwm, 4);

    // 显示右环岛PWM
    tft180_set_color((menu_item == MENU_ITEM_RIGHT_ISLAND) ? RGB565_YELLOW : RGB565_WHITE,
                     RGB565_BLACK);
    tft180_show_string(0, 68, "Right Island:");
    tft180_show_int(112, 68, menu_right_island_pwm, 4);

    // 显示高速直道PWM
    tft180_set_color((menu_item == MENU_ITEM_FAST) ? RGB565_YELLOW : RGB565_WHITE,
                     RGB565_BLACK);
    tft180_show_string(0, 90, "Fast PWM:");
    tft180_show_int(80, 90, menu_fast_pwm, 4);

    // 按键提示
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(0, 110, "S1:NEXT S2- S3+");
    tft180_show_string(0, 120, "S4:BACK");
}

/*********************************************************************************************************************
* 函数名称：Menu_DrawProfileTag
* 功能说明：显示当前正在使用的参数档案编号
********************************************************************************************************************/
static void Menu_DrawProfileTag(uint16 x, uint16 y)
{
    char text[4];

    text[0] = 'P';
    text[1] = (char)('1' + menu_profile_active);
    text[2] = menu_profile_dirty ? '*' : '\0';
    text[3] = '\0';
    tft180_show_string(x, y, text);
}

/*********************************************************************************************************************
* 函数名称：Menu_DrawProfile
* 功能说明：绘制8套参数档案的加载、保存和恢复默认页面
********************************************************************************************************************/
static void Menu_DrawProfile(void)
{
    char message[16];

    tft180_full(RGB565_BLACK);

    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    tft180_show_string(0, 0, "PROFILE");
    Menu_DrawProfileTag(136, 0);

    tft180_set_color((menu_profile_cursor == MENU_PROFILE_ITEM_SLOT) ?
                     RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 22, "SLOT");
    tft180_show_int(80, 22, (int32)(menu_profile_selected + 1), 1);
    tft180_show_string(88, 22, "/8");

    tft180_set_color((menu_profile_cursor == MENU_PROFILE_ITEM_LOAD) ?
                     RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 42, "LOAD");

    tft180_set_color((menu_profile_cursor == MENU_PROFILE_ITEM_SAVE) ?
                     RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 62, "SAVE ACTIVE");

    tft180_set_color((menu_profile_cursor == MENU_PROFILE_ITEM_DEFAULT) ?
                     RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 82, "DEFAULT");

    memset(message, 0, sizeof(message));

    if(menu_profile_notice == MENU_PROFILE_NOTICE_LOADED)
    {
        message[0] = 'L'; message[1] = 'O'; message[2] = 'A'; message[3] = 'D';
        message[4] = 'E'; message[5] = 'D'; message[6] = ' ';
        message[7] = 'P'; message[8] = (char)('1' + menu_profile_active);
    }
    else if(menu_profile_notice == MENU_PROFILE_NOTICE_SAVED)
    {
        message[0] = 'S'; message[1] = 'A'; message[2] = 'V'; message[3] = 'E';
        message[4] = 'D'; message[5] = ' ';
        message[6] = 'P'; message[7] = (char)('1' + menu_profile_active);
    }
    else if(menu_profile_notice == MENU_PROFILE_NOTICE_DEFAULT)
    {
        message[0] = 'D'; message[1] = 'E'; message[2] = 'F'; message[3] = 'A';
        message[4] = 'U'; message[5] = 'L'; message[6] = 'T'; message[7] = ' ';
        message[8] = 'P'; message[9] = (char)('1' + menu_profile_selected);
    }
    else if(menu_profile_notice == MENU_PROFILE_NOTICE_ERROR)
    {
        message[0] = 'S'; message[1] = 'A'; message[2] = 'V'; message[3] = 'E';
        message[4] = ' '; message[5] = 'E'; message[6] = 'R'; message[7] = 'R';
    }

    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 104, message);
    tft180_show_string(0, 118, "S1:NEXT S3:OK");
}

/*********************************************************************************************************************
* 函数名称：Menu_DrawLevel1
* 功能说明：绘制一级菜单，选择要进入哪个二级页面
* 参数说明：无
* 返回说明：无
* 备    注：S1移动光标；S3是确定键，选到 6 START 按 S3 就是发车
********************************************************************************************************************/
static void Menu_DrawLevel1(void)
{
    // 清屏
    tft180_full(RGB565_BLACK);

    // 标题
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    tft180_show_string(0, 0, "MENU");
    Menu_DrawProfileTag(136, 0);

    // 光标所在的那一项用黄色，其余用白色
    tft180_set_color((menu_cursor == 0) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 20, "1 IMAGE");

    tft180_set_color((menu_cursor == 1) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 38, "2 PARAMETERS");

    tft180_set_color((menu_cursor == 2) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 56, "3 MOTOR PWM");

    tft180_set_color((menu_cursor == 3) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 74, "4 DATA VIEW");

    tft180_set_color((menu_cursor == 4) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 92, "5 PROFILE");

    tft180_set_color((menu_cursor == 5) ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(8, 110, "6 START");

    // 6项菜单已经占满屏幕，不再额外显示按键提示
}

/*********************************************************************************************************************
* 函数名称：Menu_DrawImage
* 功能说明：二级页面，显示灰度图或二值化图，并叠加巡线中心线；S1切换显示模式
* 参数说明：无
* 返回说明：无
********************************************************************************************************************/
static void Menu_DrawImage(void)
{
    int i;
    const uint8 *image;
    uint8 threshold;

    // 根据当前模式选择二值化图或灰度原图
    // threshold=1：显示二值化图；threshold=0：显示灰度图
    if(menu_image_mode == 0)
    {
        image = image_two_value[0];
        threshold = 1;
    }
    else
    {
        image = mt9v03x_image[0];
        threshold = 0;
    }

    // 188x120 缩放到 160x101 显示
    tft180_show_gray_image(0, 0, image, MT9V03X_W, MT9V03X_H, 160, 101, threshold);

    // 只有二值化图显示中心线，灰度图保持原始画面
    if(menu_image_mode == 0)
    {
        for(i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--)
            tft180_draw_point(Center_Line[i] * 160 / MT9V03X_W, i * 101 / MT9V03X_H, RGB565_RED);
    }

    // 底部显示当前模式和按键提示
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    if(menu_image_mode == 0)
        tft180_show_string(0, 104, "IMAGE: BINARY");
    else
        tft180_show_string(0, 104, "IMAGE: GRAY  ");
    tft180_show_string(88, 104, "S1:SWITCH");
    tft180_show_string(0, 116, "S4:BACK");
}
/*********************************************************************************************************************
* 函数名称：Menu_DrawData
* 功能说明：二级页面，显示图像数据和各个元素标志
* 参数说明：无
* 返回说明：无
* 备    注：四个拐点显示格式是"行号+列号"，列号取 Left_Line/Right_Line 在该行的值；
*           另外显示陀螺仪积分角度 Yaw_Total，单位是度，取整数显示。
********************************************************************************************************************/
static void Menu_DrawData(void)
{
    // 整屏重画；数字用 tft180_show_int 写，它自己会用空格补齐，不会残留旧数字
    tft180_full(RGB565_BLACK);

    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);

    // 左右丢线行数
    tft180_show_string(0, 0, "LL");
    tft180_show_int(24, 0, Left_Lost_Time, 3);
    tft180_show_string(88, 0, "RL");
    tft180_show_int(112, 0, Right_Lost_Time, 3);

    // 双边丢线行数、搜索截止行
    tft180_show_string(0, 16, "BL");
    tft180_show_int(24, 16, Both_Lost_Time, 3);
    tft180_show_string(72, 16, "SSL");
    tft180_show_int(104, 16, Search_Stop_Line, 3);

    // 左右边界起始点
    tft180_show_string(0, 32, "BSL");
    tft180_show_int(32, 32, Boundry_Start_Left, 3);
    tft180_show_string(80, 32, "BSR");
    tft180_show_int(112, 32, Boundry_Start_Right, 3);

    // 左上、右上拐点：行号 + 列号
    tft180_show_string(0, 48, "LU");
    tft180_show_int(20, 48, Left_Up_Inflection_Point, 3);
    tft180_show_int(48, 48, Left_Line[Left_Up_Inflection_Point], 3);
    tft180_show_string(80, 48, "RU");
    tft180_show_int(100, 48, Right_Up_Inflection_Point, 3);
    tft180_show_int(128, 48, Right_Line[Right_Up_Inflection_Point], 3);

    // 左下、右下拐点：行号 + 列号
    tft180_show_string(0, 64, "LD");
    tft180_show_int(20, 64, Left_Down_Inflection_Point, 3);
    tft180_show_int(48, 64, Left_Line[Left_Down_Inflection_Point], 3);
    tft180_show_string(80, 64, "RD");
    tft180_show_int(100, 64, Right_Down_Inflection_Point, 3);
    tft180_show_int(128, 64, Right_Line[Right_Down_Inflection_Point], 3);

    // 环岛状态、陀螺仪角速度、陀螺仪积分角度
    tft180_show_string(0, 80, "IS");
    tft180_show_int(20, 80, Island_State, 2);
    tft180_show_string(40, 80, "GY");
    tft180_show_int(60, 80, GYRO, 3);
    tft180_show_string(88, 80, "Yaw:");
    tft180_show_int(120, 80, (int32)Yaw_Total, 4);

    // 斑马线标志、十字标志、二值化阈值
    tft180_show_string(0, 96, "ZB");
    tft180_show_int(20, 96, Zebra_Flag, 2);
    tft180_show_string(48, 96, "CR");
    tft180_show_int(68, 96, Cross_Flag, 2);
    tft180_show_string(96, 96, "TH");
    tft180_show_int(116, 96, threshold, 3);

    // 中线误差
    tft180_show_string(0, 112, "ERR");
    tft180_show_float(32, 112, Err, 5, 1);
}

/*********************************************************************************************************************
* 函数名称：Menu_Init
* 功能说明：初始化菜单
* 参数说明：无
* 返回说明：无
********************************************************************************************************************/
void Menu_Init(void)
{
    // 按键用库里的驱动：扫描周期1ms，10次采样就是10ms，点一下就能认
    key_init(1);
    key_clear_all_state();

    // CCU60_CH0 定时器 1ms 中断里调用 key_scanner()，主循环画屏再慢也不会漏按键
    // CPU0默认没有开全局中断，这里一定要打开，否则定时器中断不会进来
    interrupt_global_enable(0);
    pit_ms_init(CCU60_CH0, 1);
    pit_ms_init(CCU60_CH1, 5);
    system_start();

    // 上电默认停在一级菜单，选到 6 START 按 S3 才发车
    menu_page    = 1;
    race_started = 0;
    menu_sub    = MENU_SUB_LIST;
    menu_cursor = 0;
    menu_dirty  = 1;
    menu_item   = MENU_ITEM_P;
    Menu_ProfileInit();

    // 默认字体和颜色
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
}

/*********************************************************************************************************************
* 函数名称：Menu_DrawStop
* 功能说明：画停车提示画面（从菜单返回运行页面时用，这时还没发车，车不会动）
* 参数说明：无
* 返回说明：无
********************************************************************************************************************/
static void Menu_DrawStop(void)
{
    tft180_full(RGB565_BLACK);

    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    tft180_show_string(56, 48, "STOP");

    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(52, 96, "S4:MENU");
}

/*********************************************************************************************************************
* 函数名称：Menu_Update
* 功能说明：菜单按键处理。S4 在整个菜单里统一当“返回”用，只切界面、绝不会发车
* 参数说明：无
* 返回说明：无
* 备    注：运行页 S4进菜单；菜单里 S4返回（二级页面->一级菜单，一级菜单->运行页）；
*           一级菜单 S1移动光标 S3确定（选到6 START就是发车）；调参页 S1切换 S2减小 S3增大
********************************************************************************************************************/
void Menu_Update(void)
{
    // 运行页面：S4进入菜单
    if(menu_page == 0)
    {
        if(Menu_KeyEvent(KEY_4))
        {
            menu_page    = 1;
            menu_sub     = MENU_SUB_LIST;
            menu_cursor  = 0;
            menu_dirty   = 1;
            race_started = 0;               // 进菜单就收车，要重新发车才会跑
        }
        return;
    }

    // 菜单里 S4 统一当返回键，写在最前面，任何页面都能出来
    // 一级菜单按S4 -> 回运行页面（不发车，车停着）
    // 二级页面按S4 -> 回一级菜单
    if(Menu_KeyEvent(KEY_4))
    {
        Menu_ProfileSaveDirty();

        if(menu_sub == MENU_SUB_LIST)
        {
            menu_page  = 0;
            menu_dirty = 0;
            Menu_DrawStop();
        }
        else
        {
            menu_sub   = MENU_SUB_LIST;
            menu_dirty = 1;
        }
        return;
    }

    // 一级菜单：S1移动光标，S3确定
    if(menu_sub == MENU_SUB_LIST)
    {
        if(Menu_KeyEvent(KEY_1))
        {
            menu_cursor++;
            if(menu_cursor > 5)
                menu_cursor = 0;
            menu_dirty = 1;
        }

        if(Menu_KeyEvent(KEY_3))
        {
            if(menu_cursor == 0)
            {
                menu_sub = MENU_SUB_IMAGE;
                Gyro_RecalibrateBias();
                Island_Reset();
                menu_refresh_cnt = 0;
            }
            else if(menu_cursor == 1)
            {
                menu_sub = MENU_SUB_PARAM;
                menu_item = MENU_ITEM_P;
            }
            else if(menu_cursor == 2)
            {
                menu_sub = MENU_SUB_MOTOR;
                menu_item = MENU_ITEM_MOTOR;
            }
            else if(menu_cursor == 3)
            {
                menu_sub = MENU_SUB_DATA;
                Gyro_RecalibrateBias();
                Island_Reset();
            }
            else if(menu_cursor == 4)
            {
                menu_sub = MENU_SUB_PROFILE;
                menu_profile_cursor = MENU_PROFILE_ITEM_SLOT;
                menu_profile_selected = menu_profile_active;
            }
            else
                menu_sub = MENU_SUB_START;
            menu_dirty = 1;
        }

        if(menu_dirty && menu_sub == MENU_SUB_LIST)
        {
            menu_dirty = 0;
            Menu_DrawLevel1();
        }
    }
    // 二级页面：图像查看。S1在灰度图和二值化图之间切换，S4返回
    else if(menu_sub == MENU_SUB_IMAGE)
    {
        if(Menu_KeyEvent(KEY_1))
        {
            menu_image_mode ^= 1;
            menu_dirty = 1;
        }

        if(new_frame_flag)
        {
            new_frame_flag = 0;

            // 电机不转，但舵机照常跟着巡线走，方便用手推着车观察转向
            pwm_set_duty(SERVO_MOTOR_PWM, Servo_ControlPwm(Err));

            menu_refresh_cnt++;
            if(menu_dirty || menu_refresh_cnt >= MENU_REFRESH_DIV)
            {
                // 只在进入图像页或切换模式时清一次屏，正常刷新不清屏，避免闪烁
                if(menu_dirty)
                {
                    tft180_full(RGB565_BLACK);
                    menu_dirty = 0;
                }
                menu_refresh_cnt = 0;
                Menu_DrawImage();
            }
        }
    }
    // 二级页面：参数调整
    else if(menu_sub == MENU_SUB_PARAM)
    {
        if(Menu_KeyEvent(KEY_1))
        {
            menu_item++;
            if(menu_item >= MENU_ITEM_MOTOR)
                menu_item = MENU_ITEM_P;
            menu_dirty = 1;
        }

        if(Menu_KeyEvent(KEY_2))
            Menu_Adjust(-1);

        if(Menu_KeyEvent(KEY_3))
            Menu_Adjust(1);

        if(menu_dirty)
        {
            menu_dirty = 0;
            Menu_DrawParam();
        }
    }
    // 二级页面：电机PWM调整
    else if(menu_sub == MENU_SUB_MOTOR)
    {
        if(menu_item < MENU_ITEM_MOTOR || menu_item > MENU_ITEM_FAST)
            menu_item = MENU_ITEM_MOTOR;

        if(Menu_KeyEvent(KEY_1))
        {
            menu_item++;
            if(menu_item > MENU_ITEM_FAST)
                menu_item = MENU_ITEM_MOTOR;
            menu_dirty = 1;
        }

        if(Menu_KeyEvent(KEY_2))
            Menu_Adjust(-1);

        if(Menu_KeyEvent(KEY_3))
            Menu_Adjust(1);

        if(menu_dirty)
        {
            menu_dirty = 0;
            Menu_DrawMotor();
        }
    }
    // 二级页面：图像数据（只有S4返回，数据按帧刷新）
    else if(menu_sub == MENU_SUB_DATA)
    {
        if(new_frame_flag)
        {
            new_frame_flag = 0;

            // 电机不转，但舵机照常跟着巡线走，方便用手推着车观察转向
            pwm_set_duty(SERVO_MOTOR_PWM, Servo_ControlPwm(Err));

            menu_refresh_cnt++;
            if(menu_dirty || menu_refresh_cnt >= MENU_REFRESH_DIV)
            {
                menu_dirty       = 0;
                menu_refresh_cnt = 0;
                Menu_DrawData();
            }
        }
    }
    // 二级页面：参数档案
    else if(menu_sub == MENU_SUB_PROFILE)
    {
        if(Menu_KeyEvent(KEY_1))
        {
            menu_profile_cursor++;
            if(menu_profile_cursor >= MENU_PROFILE_ITEM_COUNT)
                menu_profile_cursor = 0;
            menu_dirty = 1;
        }

        if(menu_profile_cursor == MENU_PROFILE_ITEM_SLOT)
        {
            if(Menu_KeyEvent(KEY_2))
            {
                if(menu_profile_selected == 0)
                    menu_profile_selected = MENU_PROFILE_SLOTS - 1;
                else
                    menu_profile_selected--;
                menu_dirty = 1;
            }

            if(Menu_KeyEvent(KEY_3))
            {
                menu_profile_selected++;
                if(menu_profile_selected >= MENU_PROFILE_SLOTS)
                    menu_profile_selected = 0;
                menu_dirty = 1;
            }
        }
        else if(Menu_KeyEvent(KEY_3))
        {
            if(menu_profile_cursor == MENU_PROFILE_ITEM_LOAD)
            {
                Menu_ProfileLoadSlot(menu_profile_selected);
                menu_item = MENU_ITEM_P;
            }
            else if(menu_profile_cursor == MENU_PROFILE_ITEM_SAVE)
            {
                if(Menu_ProfileSaveDirty())
                {
                    menu_profile_notice = MENU_PROFILE_NOTICE_SAVED;
                }
                else
                {
                    menu_profile_notice = MENU_PROFILE_NOTICE_ERROR;
                }
            }
            else if(menu_profile_cursor == MENU_PROFILE_ITEM_DEFAULT)
            {
                Menu_ProfileDefaultSlot(menu_profile_selected);
                menu_item = MENU_ITEM_P;
            }
            menu_dirty = 1;
        }

        if(menu_dirty)
        {
            menu_dirty = 0;
            Menu_DrawProfile();
        }
    }
    // 发车页面：倒计时，停够裁判要求的时间后自动发车
    else
    {
        Menu_ProfileSaveDirty();

        tft180_full(RGB565_BLACK);
        tft180_set_font(TFT180_8X16_FONT);
        tft180_set_color(RGB565_CYAN, RGB565_BLACK);
        tft180_show_string(60, 32, "START");
        tft180_set_color(RGB565_WHITE, RGB565_BLACK);
        tft180_show_string(76, 64, "2");

        system_delay_ms(MENU_START_DELAY_MS / 2);
        tft180_show_string(76, 64, "1");
        system_delay_ms(MENU_START_DELAY_MS / 2);
        Gyro_RecalibrateBias();

        // 停顿结束：清按键状态、清出界标志、清斑马线标志、帧计数器归零，然后置发车标志
        key_clear_all_state();
        out_of_track_flag = 0;
        zebra_run_frame   = 0;              // 从发车这一刻开始数帧，方便计时（50帧≈1秒）
        Zebra_Flag        = 0;              // 清掉在菜单里误认的斑马线，不然一发车就停
        Island_Reset();
        race_started      = 1;

        tft180_full(RGB565_BLACK);
        tft180_set_font(TFT180_8X16_FONT);
        tft180_set_color(RGB565_CYAN, RGB565_BLACK);
        tft180_show_string(68, 56, "RUN");
        tft180_set_font(TFT180_6X8_FONT);
        tft180_set_color(RGB565_WHITE, RGB565_BLACK);
        tft180_show_string(56, 100, "S4:MENU");

        menu_page  = 0;
        menu_dirty = 1;
    }
}

/*********************************************************************************************************************
* 函数名称：Menu_GetKp
* 功能说明：返回菜单中的P参数
* 参数说明：无
* 返回说明：当前P参数
********************************************************************************************************************/
float Menu_GetKp(void)
{
    return menu_kp;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetKd
* 功能说明：返回菜单中的D参数
* 参数说明：无
* 返回说明：当前D参数
********************************************************************************************************************/
float Menu_GetKd(void)
{
    return menu_kd;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetMotorPwm
* 功能说明：返回菜单中的电机PWM
* 参数说明：无
* 返回说明：左右电机基础PWM
********************************************************************************************************************/
int Menu_GetMotorPwm(void)
{
    return menu_motor_pwm;
}

int Menu_GetLeftIslandPwm(void)
{
    return menu_left_island_pwm;
}

int Menu_GetRightIslandPwm(void)
{
    return menu_right_island_pwm;
}

int Menu_GetFastPwm(void)
{
    return menu_fast_pwm;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetKd
* 功能说明：返回菜单中的D参数
* 参数说明：无
* 返回说明：当前D参数
********************************************************************************************************************/
float Menu_GetKgyro(void)
{
    return menu_kgyro;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetExposure
* 功能说明：返回摄像头曝光度
* 参数说明：无
* 返回说明：当前曝光度，直接传给 mt9v03x_set_exposure_time()
********************************************************************************************************************/
int Menu_GetExposure(void)
{
    return menu_exposure;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetFps
* 功能说明：返回摄像头目标帧率
* 参数说明：无
* 返回说明：当前帧率，直接传给 mt9v03x_set_fps()
********************************************************************************************************************/
int Menu_GetFps(void)
{
    return menu_fps;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetDiffGain
* 功能说明：返回差速增益系数
* 参数说明：无
* 返回说明：当前差速增益系数
********************************************************************************************************************/
float Menu_GetDiffGain(void)
{
    return menu_diff_gain;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetDiffMax
* 功能说明：返回差速绝对限幅
* 参数说明：无
* 返回说明：当前差速绝对限幅，单位PWM
********************************************************************************************************************/
int Menu_GetDiffMax(void)
{
    return menu_diff_max;
}

/*********************************************************************************************************************
* 函数名称：Menu_GetDiffSlew
* 功能说明：返回差速每帧最大变化量
* 参数说明：无
* 返回说明：当前差速变化率限制，单位PWM/帧，0表示不限制
********************************************************************************************************************/
int Menu_GetDiffSlew(void)
{
    return menu_diff_slew;
}
float Menu_GetIsland_P(void)
{
    return menu_island_p;
}
float Menu_GetIsland_D(void)
{
    return menu_island_d;
}
