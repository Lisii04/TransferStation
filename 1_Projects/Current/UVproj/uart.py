import serial

def uart_send(param1, param2, param3, param4):
    ser = serial.Serial("/dev/ttyUSB0", 115200)  # 串口初始化，根据实际情况修改串口号和波特率

    ser.write(
        (
            "R"
            + "-"
            + str(param1)
            + "-"
            + str(param2)
            + "-"
            + str(param3)
            + "-"
            + str(param4)
            + "-"
            + "E\n"
        ).encode()
    )  # 发送到串口
