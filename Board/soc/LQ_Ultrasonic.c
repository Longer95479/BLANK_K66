/*-----------------------------------------------------------------------------------------------------
【平    台】龙邱K60核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】测试线性CCD
【注意事项】
-------------------------------------------------------------
超声波接口
              
GND            GND 
VCC            5V
Trig           E9
Echo           E10            
-------------------------------------------------------------
-------------------------------------------------------------------------------------------------------*/
#include "include.h"

#define HC_TRIG   PTE9   //驱动发射信号引脚
#define HC_ECHO   PTE10  //输出高电平引脚


/*------------------------------------------------------------------------------------------------------
【函    数】Test_HCSR04
【功    能】测试超声波模块
【参    数】无
【返 回 值】无
【实    例】Test_HCSR04();
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void Test_HCSR04(void)
{
	/* 初始化超声波的IO */
	GPIO_PinInit(HC_TRIG, GPO, 0);  
  	GPIO_PinInit(HC_ECHO, GPI, 0);

	LED_Init();
    UART_Init(UART4, 115200);
	systime.init();
	printf("\r\nLQ HC-SR04 Test");
	
#ifdef LQ_OLED
	OLED_Init();
    OLED_CLS();
	OLED_P8x16Str(5,0,"LQ HC-SR04 Test");
#else
	TFTSPI_Init(1);                //TFT1.8初始化  0：横屏显示  1：竖屏显示  
    TFTSPI_CLS(u16BLUE);           //清屏
	TFTSPI_P8X16Str(0,0,"LQ HC-SR04 Test",u16RED,u16BLUE);
#endif
	char txt[16];
	uint64_t now_time = systime.get_time_us();
	static uint8_t count = 0;
	uint64_t hc_time;
	float dis = 0;
	while(1)
	{
		if(systime.get_time_us() - now_time >= 20000)
		{
			now_time = systime.get_time_us();
			
			/* HC_TRIG 保持8-15us的高电平 */
			GPIO_PinWrite(HC_TRIG, 1);
			systime.delay_us(10);
			GPIO_PinWrite(HC_TRIG, 0);
			
			/* 超声波模块开始发送超声波 */
			while(!GPIO_PinRead(HC_ECHO));
			
			/* 记录高电平时长 */
			hc_time = systime.get_time_us();
			
			/* 超声波模块收到超声波反射 */
			while(GPIO_PinRead(HC_ECHO));
		
			/* 记录高电平时长 */
			hc_time = systime.get_time_us() - hc_time;
			
			/* 计算距离 hc_time/2 * 340m/s */
			dis = (float)hc_time * 0.017;          //计算距离dis
			
			if(count++ % 5 == 0)
			{
				sprintf(txt, "Dis:%5.2fcm", dis);
#ifdef LQ_OLED
				OLED_P8x16Str(0,3,txt);
#else
				TFTSPI_P8X16Str(2,3,txt,u16RED,u16BLUE);
#endif
			}
		}
	}
}