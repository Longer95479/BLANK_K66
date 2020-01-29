/*-----------------------------------------------------------------------------------------------------
【平    台】龙邱K60核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】UART例子
【注意事项】
-------------------------------------------------------------------------------------------------------*/
#include "include.h"
#include "LQ_UART.h"

/*------------------------------------------------------------------------------------------------------
【函    数】Test_Uart
【功    能】测试UART 
【参    数】无
【返 回 值】无
【实    例】Test_Uart(); //测试串口
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void Test_Uart(void)
{
    LED_Init();
    
    UART_Init(UART4, 115200);
    
    /* 优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断 */
    NVIC_SetPriority(UART4_RX_TX_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
    
    NVIC_EnableIRQ(UART4_RX_TX_IRQn);			          //使能UART4_RX_TX_IRQn的中断
    
    uint8_t count = 0;
    while(1)
    {
        printf("串口测试例程 第 %d 次打印 \n ", count++);
        UART_PutStr(UART4, "UART_PutStr is OK \n");
        LED_Reverse(0);
        delayms(1000);
    }
    
    
}