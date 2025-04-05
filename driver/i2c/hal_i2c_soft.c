#include "delay.h"
#include "stdbool.h"


//IO��������
#define SDA_IN()  {GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=0<<(1*2);}	//PC1����ģʽ
#define SDA_OUT() {GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=1<<(1*2);} 	//PC1���ģʽ
//IO����
#define IIC_SCL(n)		(n?HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET))//SCL
#define IIC_SDA(n)		(n?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET))//SDA
#define READ_SDA  		HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)//����SDA

/**
 * @brief	IIC�ײ���ʱ����
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
 * @brief	IIC��ʼ������
 *
 * @param   void
 *
 * @return  void
 */
void hal_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOC_CLK_ENABLE();   //ʹ��GPIOCʱ��
	__HAL_RCC_GPIOD_CLK_ENABLE();   //ʹ��GPIODʱ��

    /*
		SCL - PD6		SDA-PC1
	*/
    GPIO_Initure.Pin 	= GPIO_PIN_1;
    GPIO_Initure.Mode 	= GPIO_MODE_OUTPUT_PP;	//�������
    GPIO_Initure.Pull 	= GPIO_PULLUP;        	//����
    GPIO_Initure.Speed 	= GPIO_SPEED_FAST;   	//����
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);
	
	GPIO_Initure.Pin = GPIO_PIN_6;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; 	//�������
    GPIO_Initure.Pull = GPIO_PULLUP;        	//����
    GPIO_Initure.Speed = GPIO_SPEED_FAST;   	//����
    HAL_GPIO_Init(GPIOD, &GPIO_Initure);

    IIC_SDA(1);
    IIC_SCL(1);
}

/**
 * @brief	����IIC��ʼ�ź�
 *
 * @param   void
 *
 * @return  void
 */
static void hal_i2c_start(void)
{
    SDA_OUT();     //sda�����
    IIC_SDA(1);
    IIC_SCL(1);
    hal_i2c_delay();
    IIC_SDA(0);//START:when CLK is high,DATA change form high to low
	hal_i2c_delay();
    IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ��������
}
/**
 * @brief	����IICֹͣ�ź�
 *
 * @param   void
 *
 * @return  void
 */
static void hal_i2c_stop(void)
{
    SDA_OUT();//sda�����
	IIC_SDA(0);
    IIC_SCL(1);
    hal_i2c_delay();
	IIC_SDA(1);//STOP:when CLK is high DATA change form low to high
	hal_i2c_delay();
	IIC_SCL(0);//����I2C���߽����ź�
}

/**
 * @brief	�ȴ�Ӧ���źŵ���
 *
 * @param   void
 *
 * @return  uint8_t		1������Ӧ��ʧ��
 *					0������Ӧ��ɹ�
 */
static uint8_t hal_i2c_wait_ack(void)
{
    uint16_t ucErrTime = 0;
    SDA_IN();      //SDA����Ϊ����
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

    IIC_SCL(0);//ʱ�����0
    return 0;
}
/**
 * @brief	����ACKӦ��
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
 * @brief	������ACKӦ��
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
 * @brief	IIC����һ���ֽ�
 *
 * @param   txd		��Ҫ���͵�����
 *
 * @return  void
 */
static void hal_i2c_send_byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���

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
 * @brief	��1���ֽ�����
 *
 * @param   ack		1������ACK		0������nACK
 *
 * @return  uint8_t		���ض�ȡ����
 */
static uint8_t hal_i2c_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();//SDA����Ϊ����

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
        hal_i2c_nack();//����nACK
    else
        hal_i2c_ack(); //����ACK

    return receive;
}



bool hal_i2c_msg_write(uint16_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
    hal_i2c_start();
    hal_i2c_send_byte((addr << 1) | 0); //����������ַ+д����

    if(hal_i2c_wait_ack())          //�ȴ�Ӧ��
    {
        hal_i2c_stop();
        return 1;
    }

    hal_i2c_send_byte(cmd);         //д�Ĵ�����ַ
    hal_i2c_wait_ack();             //�ȴ�Ӧ��

    for(u8 i = 0; i < len; i++)
    {
        hal_i2c_send_byte(data[i]);     //��������
        hal_i2c_wait_ack();				//�ȴ�Ӧ��
    }

    hal_i2c_stop();
    return 0;
}


bool hal_i2c_msg_read(uint16_t addr, uint8_t *data, uint8_t len)
{
    hal_i2c_start();
    hal_i2c_send_byte((addr << 1) | 0x01); //����������ַ+������

    if(hal_i2c_wait_ack())          //�ȴ�Ӧ��
    {
        hal_i2c_stop();
        return 1;
    }

    for(u8 i = 0; i < len; i++)
    {
        if(i == (len - 1))
            data[i] = hal_i2c_read_byte(0);		//������,����nACK
        else
            data[i] = hal_i2c_read_byte(1);		//������,����ACK
    }

    hal_i2c_stop();
    return 0;
}
