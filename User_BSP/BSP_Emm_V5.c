#include "BSP_Emm_V5.H"



//###########################速度模式###########################







//###########################位置模式###########################

/**
  * @brief    快速绝对位置模式控制,正数表示顺时针旋转,负数表示逆时针旋转
  *             地址取1,这个项目只需要应该电机,所以地址固定1,并且多机不启用
  *             速度使用位置模块速度宏定义,加速度使用位置模式加速度宏定义, raF使用位置模式 raF宏定义, snF使用位置模式 snF宏定义
  * @param    pulse: 绝对位置目标脉冲数,范围为-2^31~2^31-1
  * @retval   地址 + 功能码 + 命令状态 + 校验字
  */
void BSP_Emm_V5_Pos_Init(void)
{
    Emm_V5_Set_QPos_Params(EMM_V5_ADDR,
                           EMM_V5_SPEED,
                           EMM_V5_ACC,
                           0x01,
                           EMM_V5_SNF);
}

void BSP_Emm_V5_Pos_Control(int32_t pulse)
{
    Emm_V5_QPos_Control(EMM_V5_ADDR, pulse);
}

