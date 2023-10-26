import serial
import string

K = 0
I = 0
P = 0

def uart_send(K,I,P):
    # ser = serial.Serial("/dev/ttyUSB0", 115200)  # 串口初始化，根据实际情况修改串口号和波特率
    
    # ser.write(
    #     (
    #         "R"
    #         + "_"
    #         + str(K)
    #         + "_"
    #         + str(I)
    #         + "_"
    #         + str(P)
    #         + "_"
    #         + "E\n"
    #     ).encode()
    # )  # 发送到串口

    print(
            "R"
            + "_"
            + str(K)
            + "_"
            + str(I)
            + "_"
            + str(P)
            + "_"
            + "E\n"
        )
    input("Data send successfully!")




while True:
    print("-------------------------")
    print("[Datas]\n[1]K:" + str(K) + "\n[2]I:" + str(I) + "\n[3]P:" + str(P))
    print("-------------------------")
    choice = input("[1.Change value|2.send data]:")
    if choice == "1":
        while True:
            buffer = input("[Change value(K,I,P)]:")
            change = buffer.split(",")
            if len(change) == 3:
                break
            print("Incorrect value!")
        if change[0] == "":
            change[0] = K
        if change[1] == "":
            change[1] = I
        if change[2] == "":
            change[2] = P
        K = float(change[0])
        I = float(change[1])
        P = float(change[2])

    else:
        print("\n\n")
        uart_send(K,I,P)