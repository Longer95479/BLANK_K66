/*-----------------------------------------------------------------------------------------------------
【平    台】龙邱K60核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】FLASH 读写例子
【注意事项】不要写靠前的扇区  会覆盖程序
-------------------------------------------------------------------------------------------------------*/
#include "include.h"


//------------------------------------------------------------------------------------------------    
//定义一个PID参数结构体  用于测试FLASH读写
//------------------------------------------------------------------------------------------------ 
typedef struct
{
    float                _kp;         //P
    float                _ki;         //I
    float                _kd;         //D
    float                _imax;       //积分限幅
    
    float                _pid_out_p;  //KP输出
    float                _pid_out_i;  //KI输出
    float                _pid_out_d;  //KD输出
    float                _pid_out;    //pid输出
    
    float                _integrator; ///< integrator value
    float                _last_error; ///< last error for derivative
    float                _last_derivative;///< last derivative for low-pass filter
    uint32_t             _last_t;    ///< last time  
    float                scaler;      //比例
}_pid_param_t;

_pid_param_t pid_param_write = 
{
	._kp             =  1.0 ,
	._ki             =  0.5 ,
	._kd             =  0.1 ,
	._imax           =   50 , 
	
	._pid_out_p      =   0,  
	._pid_out_i      =   0,  
	._pid_out_d      =   0,  
	._pid_out        =   0,
	
	._integrator     =   0, 
	._last_error     =   0, 
	._last_derivative=   0,
	._last_t         =   0,  
	.scaler          =   7,
};

/*------------------------------------------------------------------------------------------------------
【函    数】Test_Flash
【功    能】测试flash 读写
【参    数】无
【返 回 值】无
【实    例】Test_Flash(); 
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void Test_Flash(void)
{
	UART_Init(UART4, 115200);
    OLED_Init();                  //LCD初始化
    OLED_CLS();                   //LCD清屏
	KEY_Init();
	FLASH_Init();
    OLED_P8x16Str(15,0,"LQ FLASH Test"); 
    printf("\r\nLQ FLASH Test");
	printf("\r\n不要写靠前的扇区  会覆盖程序");
	printf("\r\n按下K1，对应数值加1");
	printf("\r\n按下K2，对应数值减1");
	printf("\r\n按下K0，保存当前数据");
	
	char txt[16];
	
	_pid_param_t pid_param_read;
	
	/* 从倒数第一个扇区 0偏移位置开始 读出结构体数据 */
	pid_param_read = FLASH_Read(3, 0, _pid_param_t);
	
	if(pid_param_read._kp != 1)    //第一次写
	{
		FLASH_WriteBuf(3,(const uint8 *)&pid_param_write, sizeof(pid_param_write), 0);//写入扇区
	}

	while(1)
	{
		switch(KEY_Read(0))  
        {
            case 1:
				FLASH_EraseSector(3);                                                          //擦出扇区
				FLASH_WriteBuf(3,(const uint8 *)&pid_param_read, sizeof(pid_param_read), 0);//写入扇区
                LED_Reverse(0);
                break;           
            case 2:   
				pid_param_read._ki++;
                LED_Reverse(1);
                break;
            case 3:  
				pid_param_read._ki--;
                LED_Reverse(2);
                break;
            default:
                LED_Reverse(3);
                break;
        }
		printf("Ki is : %f \n", pid_param_read._ki);
		sprintf(txt,"Ki :%5f",pid_param_read._ki);
        OLED_P8x16Str(20,6,(uint8_t*)txt);
		delayms(100);
	}

}
