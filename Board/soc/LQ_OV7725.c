/*******************************************************************************
【平    台】龙邱K66FX智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年4月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4及以上
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】100.000MHz
【pllclock】200.000MHz

******************************************************************************/
#include "include.h"


/*------------------------------------------------------------------------------------------------------
【函    数】OV7725_Init
【功    能】摄像头初始化
【参    数】fps  帧率
【返 回 值】无
【实    例】推荐使用50帧的
【注意事项】注意K60性能有限，太高帧率会出问题
--------------------------------------------------------------------------------------------------------*/
void OV7725_Init(uint8_t fps)
{     
    uint8_t id[2];    //存放摄像头id
    uint8_t ack = 0;  //检验所有寄存器操作是否出错   
    uint16_t width = OV7725_IMAGEW, height = OV7725_IMAGEH;
    uint16_t hstart, vstart, hsize;  
    //摄像头寄存器设置
    SCCB_Init();                     //初始化SCCB接口，为配置寄存器做准备
    OV7725_SoftwareReset();          //寄存器恢复初始值
    delayms(50);  
    /*7725最大分辨率 640 * 480*/
    if ((OV7725_IMAGEW > 640) || (OV7725_IMAGEH > 480))
    {
#ifdef LQ_TFT1_8
        TFTSPI_P8X8Str(0,0,"7725 dpi!!",u16RED,u16BLUE);
#else
        OLED_P6x8Str(2,1,(u8*)"7725 dpi!!");                      //摄像头识分辨率设置过大，停止运行
#endif
        while(1);
    }
    ack += SCCB_RegRead(OV7725_SCCB_ADDR,OV7725_PID_REG,&id[0]);  //读取摄像头id高八位
    ack += SCCB_RegRead(OV7725_SCCB_ADDR,OV7725_VER_REG,&id[1]);  //读取摄像头id低八位
    if(OV7725_REVISION != (((uint32_t)id[0] << 8U) | (uint32_t)id[1]))//摄像头版本寄存器 
    {     
#ifdef LQ_TFT1_8
        TFTSPI_P8X8Str(0,0,"7725 NG",u16RED,u16BLUE);
#else
        OLED_P6x8Str(2,1,(u8*)"7725 NG");                      //摄像头识别失败，停止运行
#endif
        
        while(1); 
    } 
    else                                                      //芯片ID正确
    {
#ifdef LQ_TFT1_8
        TFTSPI_P8X8Str(0,0,"7725 OK",u16RED,u16BLUE);
#else
        OLED_P6x8Str(2,1,(u8*)"7725 OK");
#endif
        
    }
    ack += OV7725_Init_Regs();     //先按官方的默认初始化 VGA 后面需要再改
    
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM2_REG, 0x03);  //输出驱动能力 Bit[4]: 睡眠模式 Bit[1:0]: 驱动能力00: 1x 01: 2x 10: 3x 11: 4x
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x46);  //  使用QVGA 和 RGB565 格式
//                                                                  /*OV7725_COM7_REG
//                                                                    Bit[7]: 重置寄存器
//                                                                    0: 不重置
//                                                                    1: 将所有寄存器重置为默认值
//                                                                    Bit[6]: 分辨率设置（7725输出支持两种分辨率，其他的任意自定义分辨率相当于截取这两种分辨率其中的一部分，会丢失视野）
//                                                                    0: VGA
//                                                                    1: QVGA
//                                                                    Bit[5]: BT.656协议开/关选择
//                                                                    Bit[4]: 传感器的原始值
//                                                                    Bit[3:2]: RGB输出格式控件
//                                                                    00: GBR4:2:2
//                                                                    01: RGB565
//                                                                    10: RGB555
//                                                                    11: RGB444
//                                                                    Bit[1:0]: 输出格式控制
//                                                                    00: YUV
//                                                                    01: Processed Bayer RAW
//                                                                    10: RGB
//                                                                    11: Bayer RAW */
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM3_REG, 0x00);//Bit[6]: 0水平镜像关 1水平镜像开  注意开启水平镜像时最好开启0x32寄存器的Bit[7]:镜像图像边缘对齐-在镜像模式下应该设置为1
//                                                                 //Bit[4]: 0 UYVY模式  1 YUYV模式  
//                                                                 //Bit[3]: 0 小端      1 大端 
//    hstart = (0x3fU << 2U);   //图像水平开始位置 使用VGA时 为0x23
//    vstart = (0x03U  << 1U) ;   //图像垂直开始位置 使用VGA时 为0x07   
    
    /*OV7725_COM7_REG
    Bit[7]: 重置寄存器
    0: 不重置
    1: 将所有寄存器重置为默认值
    Bit[6]: 分辨率设置（7725输出支持两种分辨率，其他的任意自定义分辨率相当于截取这两种分辨率其中的一部分，会丢失视野）
    0: VGA
    1: QVGA
    Bit[5]: BT.656协议开/关选择
    Bit[4]: 传感器的原始值
    Bit[3:2]: RGB输出格式控件
    00: GBR4:2:2
    01: RGB565
    10: RGB555
    11: RGB444
    Bit[1:0]: 输出格式控制
    00: YUV
    01: Processed Bayer RAW
    10: RGB
    11: Bayer RAW */
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x43);  //  使用QVGA 和 Bayer RAW 格式
                                                                  
    
    /*Bit[1:0]: Output selection
      00: YUV or RGB
      01: YUV or RGB
      10: RAW8
      11: RAW10*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_DSP_CTRL4_REG, 0x4a);  
    
    //Bit[6]: 0水平镜像关 1水平镜像开  注意开启水平镜像时最好开启0x32寄存器的Bit[7]:镜像图像边缘对齐-在镜像模式下应该设置为1
    //Bit[4]: 0 UYVY模式  1 YUYV模式  
    //Bit[3]: 大小端
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM3_REG, 0x00);
    
    /*Bit[7]: 启用快速AGC/AEC算法
    Bit[6]: AEC -步长限制
    0: 步长限于垂直空白
    1: 无限的步长
    Bit[5]: 带过滤开/关
    Bit[4]: 启用低于条带值的AEC
    0: 将曝光时间限制在1/100或1/120
    其次在任何光照条件下当启用带通滤波器
    1: 允许曝光时间小于1/100或1/120
    其次在强光条件下当启用带通滤波器
    Bit[3]: 良好的AEC开/关控制
    0: 限制最小曝光时间为一行
    1: 允许曝光时间小于一行
    Bit[2]: 自动增益控制使
    0: 手动模式
    1: 自动模式
    Bit[1]: AWB Enable
    0: 手动模式
    1: 自动模式
    Bit[0]: AEC Enable
    0: 手动模式
    1: 自动模式*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
    
    /*对比度设置 归一化值为0x20  越小对比度越大*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_CNST_REG, 0x20);
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BRIGHT_REG, 0x20);/*亮度设置*/
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SIGN_REG, 0x20);/*亮度设置*/
    hstart = 0x3fU << 2U;   //图像水平开始位置
    vstart = 0x03U << 1U;   //图像垂直开始位置

    
    OV7725_SetFPS(fps);
    
    
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_EXHCL_REG, 0x00); //虚拟像素插入LSB用于在水平方向插入虚拟像素的LSB
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_ADVFL_REG, 0x00); //垂直同步插入虚拟行(1位等于1行)的LSB
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_ADVFH_REG, 0x00); //垂直同步插入虚拟行的MSB 
    /* Resolution and timing. */
    hsize = width + 16;

    /* 设置输出图片大小. */
    
    //水平起始位置高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了10位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HSTART_REG, hstart >> 2U); 
    
    //水平宽度高位      因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了10位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HSIZE_REG, hsize >> 2U);  
    
    //垂直起始位置高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了9位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VSTART_REG, vstart >> 1U); 
    
    //垂直高度高位      因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了9位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VSIZE_REG, height >> 1U);   
    
    //水平输出宽度高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了10位数据，这里是高8位，剩下的两位放在OV7725_EXHCH_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HOUTSIZE_REG, width >> 2U); 
    
    //垂直输出高度高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了9位数据，这里是高8位，剩下的两位放在OV7725_EXHCH_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VOUTSIZE_REG, height >> 1U);
    
    //水平宽度和垂直高度的低位
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HREF_REG,
                    ((vstart & 1U) << 6U) | ((hstart & 3U) << 4U) | ((height & 1U) << 2U) | ((hsize & 3U) << 0U)); 
    
    //水平输出和垂直输出的低2位和低1位
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_EXHCH_REG, ((height & 1U) << 2U) | ((width & 3U) << 0U));  
      
}

/*------------------------------------------------------------------------------------------------------
【函    数】OV7725_SetFPS
【功    能】设置FPS 
【参    数】fps
【返 回 值】无
【实    例】
【注意事项】配置的是摄像头的帧率输出，帧率越高，单片机的新能要求越高
--------------------------------------------------------------------------------------------------------*/
void OV7725_SetFPS(uint8_t fps)
{
    if(fps > 75)
    {
        /* 锁相环8倍频 满窗口
        Bit[7:6]: 锁相环倍频器控制
        00: 不用倍频
        01: 4倍频
        10: 6倍频
        11: 8倍频
        Bit[5:4]: 自动曝光窗口大小
        00: Full window
        01: 1/2 window
        10: 1/4 window
        11: Low 2/3 window*/
        SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM4_REG, 0xC1);  //PCLK 48M   
    }
    else
    {
        SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM4_REG, 0x41);  //PCLK 24M   
    }
    uint16_t dm_lnl = 0;                                               //虚拟行
    uint8_t  div    = 1;                                               //时钟分频系数
    if(fps > 150)
    {
        fps = 150;
        div = 1;
        dm_lnl = 0;     
    }
    else if(fps > 75)
    {
        div = 1;
        dm_lnl = (uint16_t)((150 - fps) / 150.0 * (OV7725_IMAGEH + 36*(div + 1)));
    }
    else if(fps > 50)
    {
        div = 1;
        dm_lnl = (uint16_t)((75 - fps) / 75.0 * (OV7725_IMAGEH + 36*(div + 1)));
    }
    else if(fps > 37)
    {
        div = 2;
        dm_lnl = (uint16_t)((50 - fps) / 50.0 * (OV7725_IMAGEH + 36*(div + 1)));
    }
    else if(fps > 18)
    {
        div = 3;
        dm_lnl = (uint16_t)((37.5 - fps) / 37.5 * (OV7725_IMAGEH + 36*(div + 1)));
    }
    else 
    {
        div = 14;
        dm_lnl = 0;
    }
    /*时钟配置 时钟 = PCLK * 4 / （（1+1）*4） 时钟越高，帧率越快 不过DMA可能接受不了会有噪点 
    Bit[6]: 直接使用外部时钟(没有时钟预刻度可用)
    Bit[5:0]: 时钟 = PCLK × 锁相环倍频器 /[(CLKRC[5:0] + 1) × 4]*/
    SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_CLKRC_REG, div);
    
    /*虚拟行低八位， 增加虚拟行可以降低帧率，适当添加把帧率配置到自己想要的帧率*/
    SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_DM_LNL_REG, (uint8_t)dm_lnl);  
    
    /*虚拟行高八位*/
    SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_DM_LNH_REG, (uint8_t)(dm_lnl>>8)); 
    
}
/*------------------------------------------------------------------------------------------------------
【函    数】OV7725_Init_Regs
【功    能】自动曝光 VGA
【参    数】无
【返 回 值】无
【实    例】
【注意事项】
--------------------------------------------------------------------------------------------------------*/
uint8_t OV7725_Init_Regs(void)
{
    uint8_t ack = 0;
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x3d, 0x03);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x42, 0x7f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x4d, 0x09);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x64, 0xff);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x65, 0x20);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x66, 0x00);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x67, 0x48);   
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x0f, 0xc5);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x13, 0xff);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x63, 0xe0);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x14, 0x11);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x22, 0x3f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x23, 0x07);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x24, 0x40);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x25, 0x30);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x26, 0xa1);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x2b, 0x00);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x6b, 0xaa);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x0d, 0x41);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x90, 0x05);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x91, 0x01);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x92, 0x03);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x93, 0x00);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x94, 0x90);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x95, 0x8a);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x96, 0x06);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x97, 0x0b);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x98, 0x95);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x99, 0xa0);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9a, 0x1e);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9b, 0x08);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9c, 0x20);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9e, 0x81);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0xa6, 0x04);


    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x7e, 0x0c);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x7f, 0x16);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x80, 0x2a);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x81, 0x4e);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x82, 0x61);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x83, 0x6f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x84, 0x7b);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x85, 0x86);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x86, 0x8e);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x87, 0x97);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x88, 0xa4);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x89, 0xaf);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8a, 0xc5);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8b, 0xd7);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8c, 0xe8);
    return ack;
}

/*------------------------------------------------------------------------------------------------------
【函    数】OV7725_SoftwareReset
【功    能】复位摄像头 恢复默认配置
【参    数】无
【返 回 值】无
【实    例】
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void OV7725_SoftwareReset(void)
{
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x80);
}
/*------------------------------------------------------------------------------------------------------
【函    数】OV7725_LightModeConfigs
【功    能】灯光配置
【参    数】0:自动模式1:晴天2,多云3,办公室4,家里5,夜晚
【返 回 值】无
【实    例】
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void OV7725_LightModeConfigs(uint8_t mode)
{
    switch(mode)
    {
        case 1:  //晴天
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x5a);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x5c);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 2:  //多云
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x58);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x60);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
        
            break;
        case 3:  //办公室
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x84);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x4c);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 4:   //家里
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x96);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x40);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 5:   //晚上
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0xe5);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        default:   //自动模式
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
    }
}
/*------------------------------------------------------------------------------------------------------
【函    数】OV7725_SpecialEffectConfigs
【功    能】特效配置
【参    数】0:普通模式 1.黑白 2.复古  3,偏蓝4,偏红5,偏绿 6,负片
【返 回 值】无
【实    例】
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void OV7725_SpecialEffectConfigs(uint8_t mode)
{

    switch(mode)
    {
        case 1:  //黑白
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x26);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x80);
            break;
        case 2:  //复古
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x40);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0xa0);
            break;
        case 3:  //偏蓝
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0xa0);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x40);
            break;
        case 4:   //偏红
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x40);
            break;
        case 5:   //偏绿
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x60);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x60);
            break;
        case 6:   //负片
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x46);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x00);
            break;
        default:   //普通模式
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x06);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x80);
            break;
    }
}



/*********************************************************************
 *摄像头SCCB底层驱动
 *
 ***********************************************************************/



/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Init(void)
*  功能说明：配置SCCB所用引脚为GPIO功能，暂时不配置数据方向
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Init(void)
{
  GPIO_PinInit(SCCB_SCL_PIN, GPI_UP, 1);//配置为GPIO功能
//   PORTE_BASE_PTR->PCR[4] |= 0x03; //上拉
  GPIO_PinInit(SCCB_SDA_PIN, GPI_UP, 1);//配置为GPIO功能
// PORTE_BASE_PTR->PCR[5] |= 0x03; //上拉

}

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Wait(void)
*  功能说明：SCCB等待演示
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Wait(void)
{
  uint16_t i=0;
  for(i=0;i<100;i++)
  { 
    asm ("nop");
  }  
}

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Star(void)
*  功能说明：启动函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Star(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_High;
  SCL_High; 
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_Low; 
  SCCB_Wait();
}
/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Stop(void)
*  功能说明：停止函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Stop(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_High; 
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();  
}
/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：uint8 SCCB_SendByte(uint8 Data)
*  功能说明：SCCB发送字节函数
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
uint8_t SCCB_SendByte(uint8_t Data)
{
  uint8_t i;
  uint8_t Ack;
  SDA_Out;
  for( i=0; i<8; i++)
  {
    if(Data & 0x80) SDA_High;
    else            SDA_Low;    
    Data <<= 1;
    SCCB_Wait();
    SCL_High;      
    SCCB_Wait();
    SCL_Low;
    SCCB_Wait();
  }
  SDA_In;
  SCCB_Wait();
  
  SCL_High;
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;
  SCCB_Wait();
  SDA_Out;
  return Ack;
}
/*************************************************************** 

* 
* 函数名称：uint8 SCCB_ReadByte(void) 
* 功能说明：SCCB读取字节函数 
* 参数说明： 
* 函数返回：读取字节 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
uint8_t SCCB_ReadByte(void) 
{ 
  uint8_t i; 
  uint8_t byte = 0; 
  SCL_Out; 
  SDA_In; //使能输入
  for( i=0; i<8; i++) 
  { 
    SCCB_Wait(); 
    SCL_High;
    SCCB_Wait();
    byte = byte<<1;
    if(SDA_Data)
        byte++;
    SCCB_Wait();
    SCL_Low;  
  }
  SDA_Out;
  SCCB_Wait(); 
  return byte; 
} 
/*************************************************************** 

* 
* 函数名称：static void SCCB_Ack(void) 
* 功能说明：IIC有回复信号 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
static void SCCB_Ack(void) 
{ 
  SCL_Out; 
  SDA_Out;
  SCL_Low;
  SDA_Low;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 
/*************************************************************** 

* 
* 函数名称：static void SCCB_NAck(void) 
* 功能说明：IIC无回复信号 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
static void SCCB_NAck(void) 
{ 
  SCL_Out; 
  SDA_Out;
  SCL_Low;
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
*  功能说明：向设备写数据 
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
uint8_t SCCB_RegWrite(uint8 Device,uint8 Address,uint8_t Data)
{
  uint8_t Ack = 0;
  
    SCCB_Star();
    Ack = SCCB_SendByte(Device<<1);
   
    Ack = SCCB_SendByte(Address);
    
    Ack = SCCB_SendByte(Data);
    
    SCCB_Stop();
    return Ack;
}
/*************************************************************** 

* 
* 函数名称：uint8_t SCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
* 功能说明：读取数据 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
uint8_t SCCB_RegRead(uint8_t Device,uint8_t Address,uint8_t *Data) 
{   
  uint8 Ack = 0;
  Device = Device<<1;
  SCCB_Star();
  Ack += SCCB_SendByte(Device);
  SCCB_Wait();
  Ack += SCCB_SendByte(Address);
  SCCB_Wait();
  SCCB_Stop();
  SCCB_Wait();
  
  SCCB_Star();
  Ack += SCCB_SendByte(Device | 0x01);
  
  *Data = SCCB_ReadByte();
//  SCCB_Ack();
//  *Data = *Data<<8;
  
//  *Data += SCCB_ReadByte();
  SCCB_NAck();
  
  SCCB_Stop();
  
  return  Ack;
} 
/***************************************************************  
* 
* 函数名称：int SCCB_Probe(uint8_t chipAddr) 
* 功能说明：查询该地址的设备是否存在 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
int SCCB_Probe(uint8_t chipAddr) 
{ 
  uint8_t err;
  err = 0;
  chipAddr <<= 1;
  
  SCCB_Star();
  err = SCCB_SendByte(chipAddr);
  SCCB_Stop();
  
  return err;
}