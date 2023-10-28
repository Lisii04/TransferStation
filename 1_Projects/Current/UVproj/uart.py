import serial

def uart_send(param1, param2, param3):
    ser = serial.Serial("/dev/ttyUSB0", 115200)  # 串口初始化，根据实际情况修改串口号和波特率

    ser.write(
        (
            "R"
            + "_"
            + str(param1)
            + "_"
            + str(param2)
            + "_"
            + str(param3)
            + "_"
            + "E\n"
        ).encode()
    )  # 发送到串口
