import serial

def uart_send(params):
    # ser = serial.Serial("/dev/ttyUSB0", 115200)  # 串口初始化，根据实际情况修改串口号和波特率

    uart_data = "R"
    for i in params:
        uart_data += params[i]
    uart_data += "E\n"

    print(uart_data)

    # ser.write(
    #     (
    #         "R"
    #         + "_"
    #         + str(param1)
    #         + "_"
    #         + str(param2)
    #         + "_"
    #         + str(param3)
    #         + "_"
    #         + str(param4)
    #         + "_"
    #         + "E\n"
    #     ).encode()
    # )  # 发送到串口
