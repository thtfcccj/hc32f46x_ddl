/*****************************************************************************

//		                 HC32F46x的misc实现

****************************************************************************/
#include "misc.h"
#include "hc32_ddl.h" 

//-------------------------------系统时钟配置-----------------------------------
static stc_clk_sysclk_cfg_t m_stcSysclkCfg =
{
    .enHclkDiv  = ClkSysclkDiv1,  //168MHz最高200MHz,  CPU、DMA,EFM,SRAM,MPU,GPIO,DCU,INTC,QSPI
    .enExclkDiv = ClkSysclkDiv2,  // 84MHz最高100MHz， SDIOn(n=1、2) 、CAN 
    .enPclk0Div = ClkSysclkDiv1,  // 168MHz最高200MHz: Timer6 计数器用时钟 
    .enPclk1Div = ClkSysclkDiv2,  // 84MHz最高100MHz: USART,SPI,USBFS,T0,TAnT4,T6,EMB,CRC,HASH,AES,I2S
    .enPclk2Div = ClkSysclkDiv4,  // 42MHz最高60MHz,  ADC(工作) 变换时钟 
    .enPclk3Div = ClkSysclkDiv4,  // 42MHz最高50MHz,  RTC,I2,CMP,WDT、SWDT,
    .enPclk4Div = ClkSysclkDiv2,  // 84MHz最高100MHz, ADC(控制）、TRNG 
};


void SystemClockConfig(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;
    stc_sram_config_t  stcSramConfig;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);
    
    //置IO为时钟输入模式
    M4_PORT->PCRH1 = (1 << 15);
    M4_PORT->PCRH0 = (1 << 15);
    
    //Set bus clock division first
    CLK_SysClkConfig(&m_stcSysclkCfg);

    //Switch system clock source to MPLL
    //Use XTAL as MPLL source
    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    //Set MPLL out 168MHz. 
    stcMpllCfg.pllmDiv = 1u;
    //sysclk = 8M / pllmDiv * plln / PllpDiv 
    stcMpllCfg.plln    = 42u;
    stcMpllCfg.PllpDiv = 2u;
    stcMpllCfg.PllqDiv = 16u;
    stcMpllCfg.PllrDiv = 16u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    //Flash read wait cycle setting->已在FLASH初始化里实现

    //If the system clock frequency is higher than 100MHz and SRAM1, SRAM2, SRAM3 or Ret_SRAM is used,
    //   the wait cycle must be set
    #if SYS_MHZ >= 100
      stcSramConfig.u8SramIdx     = Sram12Idx | Sram3Idx | SramRetIdx;
      stcSramConfig.enSramRC      = SramCycle2;
      stcSramConfig.enSramWC      = SramCycle2;
      stcSramConfig.enSramEccMode = EccMode0;
      stcSramConfig.enSramEccOp   = SramNmi;
      stcSramConfig.enSramPyOp    = SramNmi;
      SRAM_Init(&stcSramConfig);

      CLK_MpllCmd(Enable);
    #endif //SYS_MHZ >= 100

    // Wait MPLL ready
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy)){;}

    //Set system clock source
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}



//-----------------------------------中断配置----------------------------------
//使用此函数后，需实现startup中的“IRQn_Handler”函数
void NVIC_Config(unsigned int Priority, //主优先级，3-15, 越小优先级越高！
                 IRQn_Type IRQn,         //硬件定义的中断号,0~31无限制，以上注意对应关系
                 enum en_int_src IntSrc)   //硬件中的中断源
{
  //配置中断源
  *(((volatile unsigned int *)&M4_INTC->SEL0) + IRQn) = IntSrc;
  //配置NVIC
  NVIC_ClearPendingIRQ(IRQn);  
  NVIC_SetPriority(IRQn, Priority);
  NVIC_EnableIRQ(IRQn);
}
