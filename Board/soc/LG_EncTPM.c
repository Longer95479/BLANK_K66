#include "include.h"
#include "LG_EncTPM.h"

//@brief 开启TPM12的正交解码功能
void Enc_TPM12_Init(void)
{
    //引脚初始化
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    
    /* PORTA8 is configured as TPM1_CH0 */
    PORT_SetPinMux(PORTA, 8U, kPORT_MuxAlt6);
    /* PORTA9 is configured as TPM1_CH1 */
    PORT_SetPinMux(PORTA, 9U, kPORT_MuxAlt6);
    /* PORTA10 is configured as TPM2_CH0 */
    PORT_SetPinMux(PORTA, 10U, kPORT_MuxAlt6);
    /* PORTA11 is configured as TPM2_CH1 */
    PORT_SetPinMux(PORTA, 11U, kPORT_MuxAlt6);
      
    //使能时钟
    CLOCK_EnableClock(kCLOCK_Tpm1);
    CLOCK_EnableClock(kCLOCK_Tpm2);
    
    //使能 MCGPLLCLK
    MCG_C5 |= MCG_C5_PLLCLKEN(1U);
    
    //选择 MCGPLLCLK 为 TPM 计数器时钟
    CLOCK_SetTpmClock(1U); //选择PLLFLLSELCLK 180MHz(MCGFLLCLK , or MCGPLLCLK, or IRC48M, or USB1 PFD clock as selected by SOPT2[PLLFLLSEL], and then divided by the PLLFLLCLK fractional divider as configured by SIM_CLKDIV3[PLLFLLFRAC, PLLFLLDIV]. )
    CLOCK_SetPllFllSelClock(1U, 0U, 0U);//Selects MCGPLLCLK for various peripheral clocking options
        
    //TPM初始化
    tpm_config_t tpm_config_struct;
    TPM_GetDefaultConfig(&tpm_config_struct);
    TPM_Init(TPM1, &tpm_config_struct);
    TPM_Init(TPM2, &tpm_config_struct);
    
    /* Set the timer to be in free-running mode */
    TPM1->MOD = 0xFFFF;
    TPM2->MOD = 0xFFFF;
    
    /*计数器清零*/
    TPM1->CNT = 0;
    TPM2->CNT = 0;
        
    //编码器模式初始化
    tpm_phase_params_t phA;
    phA.phaseFilterVal = 0;
    phA.phasePolarity = kTPM_QuadPhaseNormal;
    tpm_phase_params_t phB;
    phB.phaseFilterVal = 0;
    phB.phasePolarity = kTPM_QuadPhaseNormal;
    
    //使能正交解码
    TPM_SetupQuadDecode(TPM1, &phA, &phB, kTPM_QuadPhaseEncode);
    TPM_SetupQuadDecode(TPM2, &phA, &phB, kTPM_QuadPhaseEncode);
    
    //使能计数器
    TPM_StartTimer(TPM1, kTPM_SystemClock);
    TPM_StartTimer(TPM2, kTPM_SystemClock);    
    
}
  
void PIT_Init_For_IT(PITn pitn, uint32_t ms)
{
    NVIC_SetPriority(PIT0_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
    NVIC_EnableIRQ(PIT0_IRQn);
    PIT_Init(pitn, ms);
}
  
  
//在PIT中断中采集速度（和调整速度）

