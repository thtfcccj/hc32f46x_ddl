/*****************************************************************************

//		                 HC32F46x��miscʵ��

****************************************************************************/
#include "misc.h"
#include "hc32_ddl.h" 

//-------------------------------ϵͳʱ������-----------------------------------
static stc_clk_sysclk_cfg_t m_stcSysclkCfg =
{
    .enHclkDiv  = ClkSysclkDiv1,  //168MHz���200MHz,  CPU��DMA,EFM,SRAM,MPU,GPIO,DCU,INTC,QSPI
    .enExclkDiv = ClkSysclkDiv2,  // 84MHz���100MHz�� SDIOn(n=1��2) ��CAN 
    .enPclk0Div = ClkSysclkDiv1,  // 168MHz���200MHz: Timer6 ��������ʱ�� 
    .enPclk1Div = ClkSysclkDiv2,  // 84MHz���100MHz: USART,SPI,USBFS,T0,TAnT4,T6,EMB,CRC,HASH,AES,I2S
    .enPclk2Div = ClkSysclkDiv4,  // 42MHz���60MHz,  ADC(����) �任ʱ�� 
    .enPclk3Div = ClkSysclkDiv4,  // 42MHz���50MHz,  RTC,I2,CMP,WDT��SWDT,
    .enPclk4Div = ClkSysclkDiv2,  // 84MHz���100MHz, ADC(���ƣ���TRNG 
};


void SystemClockConfig(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;
    stc_sram_config_t  stcSramConfig;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);
    
    //��IOΪʱ������ģʽ
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

    //Flash read wait cycle setting->����FLASH��ʼ����ʵ��

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



//-----------------------------------�ж�����----------------------------------
//ʹ�ô˺�������ʵ��startup�еġ�IRQn_Handler������
void NVIC_Config(unsigned int Priority, //�����ȼ���3-15, ԽС���ȼ�Խ�ߣ�
                 IRQn_Type IRQn,         //Ӳ��������жϺ�,0~31�����ƣ�����ע���Ӧ��ϵ
                 enum en_int_src IntSrc)   //Ӳ���е��ж�Դ
{
  //�����ж�Դ
  *(((volatile unsigned int *)&M4_INTC->SEL0) + IRQn) = IntSrc;
  //����NVIC
  NVIC_ClearPendingIRQ(IRQn);  
  NVIC_SetPriority(IRQn, Priority);
  NVIC_EnableIRQ(IRQn);
}
