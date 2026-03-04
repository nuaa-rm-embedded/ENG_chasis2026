#include "Includes.h"

KEYS Keys;

void Single_Key_Init(KEY_MSG *msg, uint16_t key_code)
{
    msg->code        = key_code;
    msg->press_flag  = 0;
    msg->toggle_flag = 0;
}

// 键盘16个按键初始化
void Key_Init(void)
{
    uint8_t i = 0;
    KEY_MSG *pMsg = &Keys.KEY_W; //*pMsg为指向Keys(键盘按键数据结构体)的结构体指针
    for (i = 0; i < 16; i++)
        Single_Key_Init(pMsg + i, 1 << i); // Keys.code分别赋值1~16
}

/// @brief	键按下 ? 1(仅一次) : 0
bool Key_Check_Press(KEY_MSG *msg)
{
    // 上次没按下，这次按下(检测按下)
    if ((RC_CtrlData.key.info & msg->code) && !msg->press_flag) {
        msg->press_flag = 1;
        return 1;
    }
    // 上次按下，这次没按下(检测松手)
    if (!(RC_CtrlData.key.info & msg->code) && msg->press_flag)
        msg->press_flag = 0;
    return 0;
}

/// @brief	键按下 ? 1 : 0
bool Key_Check_Hold(KEY_MSG *msg)
{
    if ((RC_CtrlData.key.info & msg->code) && !msg->press_flag)
        msg->press_flag = 1;
    if (!(RC_CtrlData.key.info & msg->code) && msg->press_flag)
        msg->press_flag = 0;
    return msg->press_flag;
}

/// @brief	键按下 ? 标志位^=1(仅一次)
bool Key_Check_Toggle(KEY_MSG *msg)
{
    if (Key_Check_Press(msg))
        msg->toggle_flag ^= 1;
    return msg->toggle_flag;
}
