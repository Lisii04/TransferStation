import serial


class uart_data:
    def __init__(self):
        self.IF_STOP = ""
        self.IF_SLOW = ""
        self.IF_READY = ""
        self.delta_x = ""


def uart_send(delta_x, IF_READY, IF_SLOW, IF_STOP):
    ser = serial.Serial("/dev/ttyUSB0", 115200)  # 串口初始化，根据实际情况修改串口号和波特率

    data1 = uart_data()

    data1.delta_x = round(delta_x,3)
    data1.IF_READY = IF_READY
    data1.IF_SLOW = IF_SLOW
    data1.IF_STOP = IF_STOP

    

    ser.write(
        (
            "R"
            + "-"
            + str(data1.IF_READY)
            + "-"
            + str(data1.IF_SLOW)
            + "-"
            + str(data1.IF_STOP)
            + "-"
            + str(data1.delta_x)
            + "-"
            + "E\n"
        ).encode()
    )  # 发送到串口
