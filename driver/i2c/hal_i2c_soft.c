#include "delay.h"
#include "stdbool.h"


//IO方向设置
#define SDA_IN()  {GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=0<<(1*2);}	//PC1输入模式
#define SDA_OUT() {GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=1<<(1*2);} 	//PC1输出模式
//IO操作
#define IIC_SCL(n)		(n?HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET))//SCL
#define IIC_SDA(n)		(n?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET))//SDA
#define READ_SDA  		HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)//输入SDA

/**
 * @brief	IIC底层延时函数
 *
 * @param   void
 *
 * @return  void
 */
void hal_i2c_delay(void)
{
	delay_us(500);
}

/**
 * @brief	IIC初始化函数
 *
 * @param   void
 *
 * @return  void
 */
void hal_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOC_CLK_ENABLE();   //使能GPIOC时钟
	__HAL_RCC_GPIOD_CLK_ENABLE();   //使能GPIOD时钟

    /*
		SCL - PD6		SDA-PC1
	*/
    GPIO_Initure.Pin 	= GPIO_PIN_1;
    GPIO_Initure.Mode 	= GPIO_MODE_OUTPUT_PP;	//推挽输出
    GPIO_Initure.Pull 	= GPIO_PULLUP;        	//上拉
    GPIO_Initure.Speed 	= GPIO_SPEED_FAST;   	//快速
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);
	
	GPIO_Initure.Pin = GPIO_PIN_6;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; 	//推挽输出
    GPIO_Initure.Pull = GPIO_PULLUP;        	//上拉
    GPIO_Initure.Speed = GPIO_SPEED_FAST;   	//快速
    HAL_GPIO_Init(GPIOD, &GPIO_Initure);

    IIC_SDA(1);
    IIC_SCL(1);
}

/**
 * @brief	产生IIC起始信号
 *
 * @param   void
 *
 * @return  void
 */
static void hal_i2c_start(void)
{
    SDA_OUT();     //sda线输出
    IIC_SDA(1);
    IIC_SCL(1);
    hal_i2c_delay();
    IIC_SDA(0);//START:when CLK is high,DATA change form high to low
	hal_i2c_delay();
    IIC_SCL(0);//钳住I2C总线，准备发送或接收数据
}
/**
 * @brief	产生IIC停止信号
 *
 * @param   void
 *
 * @return  void
 */
static void hal_i2c_stop(void)
{
    SDA_OUT();//sda线输出
	IIC_SDA(0);
    IIC_SCL(1);
    hal_i2c_delay();
	IIC_SDA(1);//STOP:when CLK is high DATA change form low to high
	hal_i2c_delay();
	IIC_SCL(0);//发送I2C总线结束信号
}

/**
 * @brief	等待应答信号到来
 *
 * @param   void
 *
 * @return  uint8_t		1，接收应答失败
 *					0，接收应答成功
 */
static uint8_t hal_i2c_wait_ack(void)
{
    uint16_t ucErrTime = 0;
    SDA_IN();      //SDA设置为输入
    IIC_SDA(1);
    hal_i2c_delay();
    IIC_SCL(1);
    hal_i2c_delay();

    while(READ_SDA)
    {
        ucErrTime++;
		
        if(ucErrTime > 2000)
        {
            hal_i2c_stop();
            return 1;
        }
    }

    IIC_SCL(0);//时钟输出0
    return 0;
}
/**
 * @brief	产生ACK应答
 *
 * @param   void
 *
 * @return  void
 */
static void hal_i2c_ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    hal_i2c_delay();
    IIC_SCL(1);
    hal_i2c_delay();
    IIC_SCL(0);
}
/**
 * @brief	不产生ACK应答
 *
 * @param   void
 *
 * @return  void
 */
static void hal_i2c_nack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    hal_i2c_delay();
    IIC_SCL(1);
    hal_i2c_delay();
    IIC_SCL(0);
}
/**
 * @brief	IIC发送一个字节
 *
 * @param   txd		需要发送的数据
 *
 * @return  void
 */
static void hal_i2c_send_byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL(0);//拉低时钟开始数据传输

    for(t = 0; t < 8; t++)
    {
        IIC_SDA((txd & 0x80) >> 7);
        txd <<= 1;
        IIC_SCL(1);
        hal_i2c_delay();
        IIC_SCL(0);
        hal_i2c_delay();
    }
}
/**
 * @brief	读1个字节数据
 *
 * @param   ack		1，发送ACK		0，发送nACK
 *
 * @return  uint8_t		返回读取数据
 */
static uint8_t hal_i2c_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();//SDA设置为输入

    for(i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        hal_i2c_delay();
        IIC_SCL(1);
        receive <<= 1;
        if(READ_SDA)receive++;
        hal_i2c_delay();
    }

    if(!ack)
        hal_i2c_nack();//发送nACK
    else
        hal_i2c_ack(); //发送ACK

    return receive;
}



bool hal_i2c_msg_write(uint16_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
    hal_i2c_start();
    hal_i2c_send_byte((addr << 1) | 0); //发送器件地址+写命令

    if(hal_i2c_wait_ack())          //等待应答
    {
        hal_i2c_stop();
        return 1;
    }

    hal_i2c_send_byte(cmd);         //写寄存器地址
    hal_i2c_wait_ack();             //等待应答

    for(u8 i = 0; i < len; i++)
    {
        hal_i2c_send_byte(data[i]);     //发送数据
        hal_i2c_wait_ack();				//等待应答
    }

    hal_i2c_stop();
    return 0;
}


bool hal_i2c_msg_read(uint16_t addr, uint8_t *data, uint8_t len)
{
    hal_i2c_start();
    hal_i2c_send_byte((addr << 1) | 0x01); //发送器件地址+读命令

    if(hal_i2c_wait_ack())          //等待应答
    {
        hal_i2c_stop();
        return 1;
    }

    for(u8 i = 0; i < len; i++)
    {
        if(i == (len - 1))
            data[i] = hal_i2c_read_byte(0);		//读数据,发送nACK
        else
            data[i] = hal_i2c_read_byte(1);		//读数据,发送ACK
    }

    hal_i2c_stop();
    return 0;
}
