/*---------------------------------------------------------------
【平    台】龙邱K60核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】OV7725测试
【注意事项】
----------------------------------------------------------------*/
#include "include.h"
#include "LQ_OV7725.h"

uint8_t Image_Data[IMAGEH][IMAGEW];      //图像原始数据存放
uint8_t Image_Use[LCDH][LCDW];           //压缩后之后用于存放屏幕显示数据
uint8_t Pixle[LCDH][LCDW];               //二值化后用于OLED显示的数据
uint8_t  Line_Cont=0;                    //行计数
uint8_t  Field_Over_Flag=0;              //场标识


/*---------------------------------------------------------------
【函    数】PORTC_Interrupt
【功    能】PORTC端口的中断服务函数 用于摄像头行场中断
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PORTD_IRQHandler(void)
{     
  //行中断PTD14
  int n;    
    n=14;   //行中断
  if((PORTD_ISFR & (1<<n) ))//行中断 (1<<14)
  {    
    PORTD_ISFR |= (1<<n);   //清除中断标识
    // 用户程序            
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[Line_Cont][0]));   //开启DMA传输 
    if(Line_Cont > IMAGEH)  //采集结束
    { 
      Line_Cont=0; 
      return ;
    } 
    ++Line_Cont;            //行计数
    return ; 
  }
  //场中断PTD15
  n=15;  //场中断
  if((PORTD_ISFR & (1<<n)))//(1<<15)
  {
    PORTD_ISFR |= (1<<n);  //清除中断标识
    // 用户程序 
    Line_Cont = 0;         //行计数清零
    Field_Over_Flag=1;     //场结束标识
  } 
}


/*---------------------------------------------------------------
【函    数】Test_OV7725
【功    能】测试OV7725 在OLED上显示图像
【参    数】无
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
void Test_OV7725(void)
{  
    OLED_Init();
    OLED_CLS();
    OLED_Show_Frame80();      //画图像 LCDW*LCDH 外框
    
    /*  摄像头初始化 */
    OV7725_Init(50);          //摄像头初始化
    
    //摄像头有关接口初始化
    GPIO_ExtiInit(PTD14,rising_down);   //行中断
    GPIO_ExtiInit(PTD15,falling_up);    //场中断  
    
    /* 优先级配置 抢占优先级0  子优先级2   越小优先级越高  抢占优先级可打断别的中断 */
    NVIC_SetPriority(PORTD_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,2));
    NVIC_EnableIRQ(PORTD_IRQn);			         //使能PORTD_IRQn的中断 
    
    //八位数据输入口
    GPIO_PinInit(PTD0,GPI,0);                
    GPIO_PinInit(PTD1,GPI,0);
    GPIO_PinInit(PTD2,GPI,0);
    GPIO_PinInit(PTD3,GPI,0);
    GPIO_PinInit(PTD4,GPI,0);
    GPIO_PinInit(PTD5,GPI,0);
    GPIO_PinInit(PTD6,GPI,0);
    GPIO_PinInit(PTD7,GPI,0);     
    
    //初始化DMA采集       通道4        PTD0-PTD7           目的地址      触发信号 每次一个BYTE  长度    下拉上升沿触发
    DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, (IMAGEW ), DMA_rising_down); 
    /* 摄像头初始化结束 */
    
    while(1)
    { 
        LED_Reverse(1);           //LED指示程序运行状态
        if(Field_Over_Flag)       //完成一场图像采集
        { 
            Get_Use_Image();      //从采集图像数据中取出自己想要使用的大小
            Get_01_Value(1);      //二值化图像数据

            Draw_Road();          //龙邱OLED模块显示动态图像
            Field_Over_Flag= 0;       
        }    
    }
}


/*---------------------------------------------------------------
【函    数】Get_Use_Image
【功    能】获取需要使用的图像大小
【参    数】无
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
__ramfunc void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 0;
  int div_h = IMAGEH/LCDH;
  int div_w = IMAGEW/LCDW;
  for(i = 0; i  < IMAGEH; i+=div_h)  //240行，每4行采集一行，
  {
    for(j = 0;j < IMAGEW; j+=div_w)  //320 / 4  = 80，
    {        
      Image_Use[row][line] = Image_Data[i][j];         
      line++;        
    }      
    line = 0;
    row++;      
  }  
}

/*---------------------------------------------------------------
【函    数】Get_01_Value
【功    能】二值化
【参    数】mode  ：  0：使用大津法阈值    1：使用平均阈值
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
void Get_01_Value(uint8_t mode)
{
  int i = 0,j = 0;
  uint8_t Threshold;
  uint32_t  tv=0;
  char txt[16];
  if(mode)
  {
      //累加
      for(i = 0; i <LCDH; i++)
      {    
          for(j = 0; j <LCDW; j++)
          {                            
              tv+=Image_Use[i][j];   //累加  
          } 
      }
      Threshold=tv/LCDH/LCDW;        //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
      Threshold=Threshold*7/10+10;   //此处阈值设置，根据环境的光线来设定 
  }
  else
  {
      Threshold = GetOSTU(Image_Use);//大津法阈值
      Threshold = (uint8_t)(Threshold * 0.5) + 70;
  } 
  sprintf(txt,"%03d",Threshold);//前者为大津法求得的阈值，后者为平均值  
  OLED_P6x8Str(80,1,(u8*)txt);
  
  for(i = 0; i < LCDH; i++)
  {
    for(j = 0; j < LCDW; j++)
    {                                
      if(Image_Use[i][j] >Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来    
        Pixle[i][j] =1;        
      else                                        
        Pixle[i][j] =0;
    }    
  }
}


/*---------------------------------------------------------------
【函    数】Draw_Road
【功    能】在OLED上画出摄像头的图像
【参    数】无
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
void Draw_Road(void)
{ 	 
  u8 i = 0, j = 0,temp=0;
  
  //发送帧头标志
  for(i=8;i<56;i+=8)//6*8=48行 
  {
    OLED_Set_Pos(24,i/8+1);//起始位置
    for(j=0;j<LCDW;j++)  //列数
    { 
      temp=0; 
      if(Pixle[0+i][j]) temp|=1;
      if(Pixle[1+i][j]) temp|=2;
      if(Pixle[2+i][j]) temp|=4;
      if(Pixle[3+i][j]) temp|=8;
      if(Pixle[4+i][j]) temp|=0x10;
      if(Pixle[5+i][j]) temp|=0x20;
      if(Pixle[6+i][j]) temp|=0x40;
      if(Pixle[7+i][j]) temp|=0x80;
      OLED_WrDat(temp); 	  	  	  	  
    }
  }  
}


/*---------------------------------------------------------------
【函    数】Pixle_Filter
【功    能】过滤噪点
【参    数】无
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
void Pixle_Filter(void)
{  
  int nr; //行
  int nc; //列
  
  for(nr=1; nr<LCDH-1; nr++)
  {  	    
    for(nc=1; nc<LCDW-1; nc=nc+1)
    {
      if((Pixle[nr][nc]==0)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]>2))         
      {
        Pixle[nr][nc]=1;
      }	
      else if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))         
      {
        Pixle[nr][nc]=0;
      }	
    }	  
  }  
}



/***************************************************************
* 
* 函数名称：SendPicture 
* 功能说明：发送图像到上位机 ，不同的上位机注意修改对应的数据接收协议
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
***************************************************************/ 
void UARTSendPicture(uint8_t  tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  UART_PutChar(UART4,0x01); //发送帧头标志 WindowsFormsApplication1.exe
  UART_PutChar(UART4,0xfe); //发送帧头标志 WindowsFormsApplication1.exe
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW ;j++) 
    { 
      if(tmImage[i][j]==0xfe) 
      { 
        tmImage[i][j]=0xff; //防止发送标志位 
      } 
      UART_PutChar(UART4,tmImage[i][j]); 
    } 
  }
  UART_PutChar(UART4,0xfe); //发送帧尾标志 
  UART_PutChar(UART4,0x01); //发送帧尾标志 
} 

/*************************************************************** 
* 
* 函数名称：uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
* 功能说明：求阈值大小 
* 参数说明： 
* 函数返回：阈值大小 
* 修改时间：2018年3月27日 
* 备 注： 
参考：https://blog.csdn.net/zyzhangyue/article/details/45841255
      https://www.cnblogs.com/moon1992/p/5092726.html
      https://www.cnblogs.com/zhonghuasong/p/7250540.html     
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7）将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
***************************************************************/ 
uint8_t GetOSTU(uint8_t tmImage[LCDH][LCDW]) 
{ 
  int16_t i,j; 
  uint32_t Amount = 0; 
  uint32_t PixelBack = 0; 
  uint32_t PixelIntegralBack = 0; 
  uint32_t PixelIntegral = 0; 
  int32_t PixelIntegralFore = 0; 
  int32_t PixelFore = 0; 
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差; 
  int16_t MinValue, MaxValue; 
  uint8_t Threshold = 0;
  uint8_t HistoGram[256];              //  

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图 
  
  for (j = 0; j < LCDH; j++) 
  { 
    for (i = 0; i < LCDW; i++) 
    { 
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值
      
  if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色    
  if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];   //前景像素点数
    PixelFore = Amount - PixelBack;         //背景像素点数
    OmegaBack = (double)PixelBack / Amount;//前景像素百分比
    OmegaFore = (double)PixelFore / Amount;//背景像素百分比
    PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
    MicroBack = (double)PixelIntegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (double)PixelIntegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
} 