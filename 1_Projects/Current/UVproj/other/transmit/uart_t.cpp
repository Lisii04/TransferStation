#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

int uartfd;

typedef struct uart_data
{
    int16_t IF_READY;
    int16_t IF_SLOW;
    int16_t IF_STOP;
    float delta_x;
} *ptr_uart_data;

/**
  * @brief  将数据段重组为uint8类型的数组
  * @param
        infoSeg 指向一个uart_data的指针
  * @param
        infoArray 由数据段重组的uint8类型数组
  * @retval 无
  */
void Info_to_Array_uint8(ptr_uart_data infoSeg, uint8_t *infoArray)
{
    int ptr = 0;
    uint8_t
        *infoElem = (uint8_t *)infoSeg;
    for (ptr = 0; ptr < sizeof(uart_data); ptr++)
    {
        infoArray[ptr] = (*(infoElem + ptr));
    }
}

/**
  * @brief  按协议打包
  * @param
        infopackage 打包结果
    @param
        infoArray 由数据段重组的uint8类型数组
    @param
        infoSize 数据段的大小--占用内存字节数（协议规定为32Byte）
  * @retval 无
  */
void Info_Pack(uint8_t *infopackage, uint8_t *infoArray, uint8_t infoSize)
{
    uint8_t ptr = 0;
    infopackage[0] = 0x80;
    infopackage[1] = 0x81;
    infopackage[2] = 0x82;

    /* 将信息封如入数据包中 */
    for (; ptr < infoSize; ptr++)
    {
        infopackage[ptr + 3] = infoArray[ptr];
    }

    infopackage[ptr + 3] = 0x82;
    infopackage[ptr + 4] = 0x81;
    infopackage[ptr + 5] = 0x80;
}

/**
 * @brief  通过USART通道向上位机发送一个字节（8bit）的数据
 * @param  byte 要发送的8位数据
 * @retval 无
 */
void UART_SendByte(uint8_t byte, int serialPort)
{
    int n = write(serialPort, &byte, sizeof(byte));
    if (n < 0)
    {
        perror("Error writing to serial port");
    }
}

/**
* @brief  将数据包发送
  * @param
        infoPackage 数据包
    @param
        packSize 数据包的大小--占用内存字节数（协议规定为38Byte）
  * @retval 无
  */
void Info_SendPack(uint8_t *infoPackage, uint8_t packSize, int serialPort)
{
    int ptr = 0;
    for (ptr = 0; ptr < packSize; ptr++)
    {
        UART_SendByte(infoPackage[ptr], serialPort);
    }
}

/**
* @brief  将数据打包并发送
  * @param
            ptr_uart_data 指向一个装填好信息的的指针
  * @retval 无
  */
void Info_Pack_and_Send(ptr_uart_data ptr_uart_data, int serialPort)
{
    uint8_t infoArray[sizeof(uart_data)] = {0};
    uint8_t infoPackage[sizeof(uart_data) + 6] = {0};
    Info_to_Array_uint8(ptr_uart_data, infoArray);
    Info_Pack(infoPackage, infoArray, sizeof(infoArray));
    Info_SendPack(infoPackage, sizeof(infoPackage), serialPort);
}

/**
* @brief  串口初始化函数
  * @param
            serial_name 串口名
  * @retval serialPort
  */
int serial_init(char *serial_name)
{
    int serialPort;
    serialPort = open(serial_name, O_RDWR);
    if (serialPort == -1)
    {
        perror("Error opening serial port");
        return -1;
    }

    // 配置串口参数
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serialPort, &tty) != 0)
    {
        perror("Error configuring serial port");
        return -1;
    }

    cfsetospeed(&tty, B115200); // 设置波特率为115200
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD); // 启用本地连接和接收使能

    tty.c_cflag &= ~PARENB; // 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB; // 1个停止位
    tty.c_cflag &= ~CSIZE;  // 清除数据位设置
    tty.c_cflag |= CS8;     // 设置数据位为8位

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
    {
        perror("Error configuring serial port");
        return -1;
    }

    return serialPort;
}

int main()
{
    std::string port_name = "/dev/ttyUSB0";
    int serialPort = serial_init((char *)port_name.data());

    ptr_uart_data package;

    package->IF_READY = 1;
    package->IF_SLOW = 1;
    package->IF_STOP = 1;
    package->delta_x = 1.0141421412;

    Info_Pack_and_Send(package, serialPort);

    // 关闭串口
    close(serialPort);

    std::cout << "send" << std::endl;

    return 0;
}
