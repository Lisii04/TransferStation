import numpy
from PIL import Image
import tkinter as tk
import tkinter.filedialog
import os
import matplotlib.pyplot
import keyboard


def selectPath():
    # 选择文件path_接收文件地址
    path_ = tk.filedialog.askdirectory()

    # 通过replace函数替换绝对文件地址中的/来使文件可被程序读取
    path_ = path_.replace("\\", "/")
    # path设置path_的值
    return path_


print(">Select dir ：[Enter]")

IMAGES_FILE_PATH = selectPath()

print(">Dir selected ： " + IMAGES_FILE_PATH)

fl = open("Train.csv", "a+")

root = tk.Tk()

files = os.listdir(IMAGES_FILE_PATH)  # 得到文件夹下的所有文件名称
s = []
for file in files:  # 遍历文件夹
    # file = open('test.csv', 'a+')
    IMAGE_FILE_PATH = IMAGES_FILE_PATH + "/" + file

    img = Image.open(IMAGE_FILE_PATH)

    small_img = img.resize((28, 28))

    img_array = numpy.array(small_img.convert('L'))

    img_array = 255 - img_array

    matplotlib.pyplot.imshow(img_array, cmap='Greys', interpolation='None')
    matplotlib.pyplot.waitforbuttonpress()

    while True:
        event = keyboard.read_event()
        if event.event_type == keyboard.KEY_DOWN:
            break

    print(event.name)
    fl.write(event.name)

    img_data = numpy.reshape(img_array, 784)

    for i in img_data:
        fl.write("," + str(img_data[i]))

    fl.write("\n")
