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
    int IF_READY;
    int IF_SLOW;
    int IF_STOP;
    float delta_x;
} *ptr_uart_data;

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

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
    tty.c_oflag &= ~OPOST;                          /*Output*/

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
    std::string port_name = "/dev/ttyUSB1";
    int serialPort = serial_init((char *)port_name.data());

    ptr_uart_data package;

    char receive_data[38] = {0};

    int readData = 0;

    std::cout << "----------" << std::endl;

    while (1) // 循环读取数据
    {
        if ((readData = read(serialPort, receive_data, 38)) > 0)
        {
            break;
        }
    }

    uint8_t infoArray[sizeof(receive_data) - 6] = {0};

    for (int i = 0; i < sizeof(receive_data) - 6; i++)
    {
        infoArray[i] = receive_data[i + 3];
    }

    for (int i = 0; i < sizeof(infoArray); i++)
    {
        printf("%d", infoArray[i]);
    }

    // for (int i = 0; i < sizeof(infoArray); i++)
    //     *((uint8_t *)package + i) = infoArray[i];

    // printf("\n");

    // std::cout << package->IF_STOP << "\n";

    // 关闭串口
    close(serialPort);

    return 0;
}
