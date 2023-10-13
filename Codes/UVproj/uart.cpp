#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

int main()
{
    const char *serialPort_TX = "/dev/ttyAMA0"; // 串口设备路径，根据实际情况修改
    const char *serialPort_RX = "/dev/ttyAMA0"; // 串口设备路径，根据实际情况修改
    const speed_t baudRate = B115200;           // 波特率，根据实际情况修改

    // 打开串口设备
    int serial_fd_tx = open(serialPort_TX, O_RDWR | O_NOCTTY | O_NDELAY);
    int serial_fd_rx = open(serialPort_RX, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_tx == -1 || serial_fd_rx == -1)
    {
        std::cerr << "can not open serial device!" << std::endl;
        return 1;
    }

    // 配置串口
    struct termios serialConfig;
    tcgetattr(serial_fd_tx, &serialConfig);
    cfsetispeed(&serialConfig, baudRate);
    cfsetospeed(&serialConfig, baudRate);
    serialConfig.c_cflag |= (CLOCAL | CREAD);
    serialConfig.c_cflag &= ~PARENB; // 无奇偶校验
    serialConfig.c_cflag &= ~CSTOPB; // 1位停止位
    serialConfig.c_cflag &= ~CSIZE;
    serialConfig.c_cflag |= CS8; // 8位数据位
    tcsetattr(serial_fd_tx, TCSANOW, &serialConfig);

    // 定义要输出的数字
    int num = 196;

    while (true)
    {
        // 发送数字到串口
        char buffer[32];
        int bytes_written = snprintf(buffer, sizeof(buffer), "%d", num);
        ssize_t bytes_sent = write(serial_fd_tx, buffer, static_cast<size_t>(bytes_written));
        if (bytes_sent < 0)
        {
            break;
        }

        num++;
        if (num > 205)
        {
            num = 196;
        }

        char receive_buffer[128];
        ssize_t bytes_read = read(serial_fd_rx, receive_buffer, sizeof(receive_buffer) - 1);
        if (bytes_read > 0)
        {
            receive_buffer[bytes_read] = '\0'; // 添加字符串终止符
            std::cout << "接收到的数据: " << receive_buffer << std::endl;
        }

        usleep(200000); // 等待200毫秒
    }

    close(serial_fd_tx);
    close(serial_fd_rx); // 关闭串口
    return 0;
}
