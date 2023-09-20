import numpy
import matplotlib.pyplot
import scipy.special
from PIL import Image
import tkinter as tk
import tkinter.filedialog

inputnodes = 784
hiddennodes = 100
outputnodes = 10
learningrate = 0.2


class NeuralNetwork:
    def __init__(self, inputnodes, hiddennodes, outputnodes, learningrate):
        self.input = inputnodes
        self.hidden = hiddennodes
        self.output = outputnodes

        self.lr = learningrate

        self.wih = numpy.random.normal(0.0, pow(self.hidden, -0.5), (self.hidden, self.input))
        self.who = numpy.random.normal(0.0, pow(self.output, -0.5), (self.output, self.hidden))

        pass

    def train(self, inputs_list, targets_list):
        inputs = numpy.array(inputs_list, ndmin=2).T
        targets = numpy.array(targets_list, ndmin=2).T

        hidden_inputs = numpy.dot(self.wih, inputs)
        hidden_outputs = self.activation_function(hidden_inputs)

        final_inputs = numpy.dot(self.who, hidden_outputs)
        final_outputs = self.activation_function(final_inputs)

        output_errors = targets - final_outputs
        hidden_errors = numpy.dot(self.who.T, output_errors)
        self.who += self.lr * numpy.dot((output_errors * final_outputs * (1.0 - final_outputs)),
                                        numpy.transpose(hidden_outputs)
                                        )
        self.wih += self.lr * numpy.dot((hidden_errors * hidden_outputs * (1.0 - hidden_outputs)),
                                        numpy.transpose(inputs)
                                        )
        pass

    def query(self, inputs):
        hidden_inputs = numpy.dot(self.wih, inputs)
        hidden_outputs = self.activation_function(hidden_inputs)
        final_inputs = numpy.dot(self.who, hidden_outputs)
        final_outputs = self.activation_function(final_inputs)

        return final_outputs

    def activation_function(self, x):
        return scipy.special.expit(x)

    def get_who(self):
        return self.who

    def get_wih(self):
        return self.wih

    def set_data(self, who, wih):
        self.who = who
        self.wih = wih


who = numpy.load('who.npy')

wih = numpy.load('wih.npy')

# INIT
rows = inputnodes
columns = hiddennodes

n = NeuralNetwork(inputnodes, hiddennodes, outputnodes, learningrate)

n.set_data(who, wih)


# USE
def selectPath():
    # 选择文件path_接收文件地址
    path_ = tk.filedialog.askopenfilename()

    # 通过replace函数替换绝对文件地址中的/来使文件可被程序读取
    # 注意：\\转义后为\，所以\\\\转义后为\\
    path_ = path_.replace("\\", "/")
    # path设置path_的值
    return path_


while 1:
    print(">Select image ：[Enter]")

    input()

    IMAGE_FILE_PATH = selectPath()

    print(">Image selected ： " + IMAGE_FILE_PATH)

    img = Image.open(IMAGE_FILE_PATH)

    small_img = img.resize((28, 28))

    img_array = numpy.array(small_img.convert('L'))

    img_array = 255 - img_array

    # matplotlib.pyplot.imshow(img_array, cmap='Greys', interpolation='None')
    # matplotlib.pyplot.waitforbuttonpress()

    img_data = numpy.reshape(img_array, 784)

    img_data = (img_data / 255.0 * 0.99) + 0.01

    results = n.query(img_data)

    result = numpy.argmax(results)

    i = 0
    print("|>Possibilities : ")
    for j in results:
        print(">" + str(i) + " : {:.2%}".format(results[i]))
        i += 1

    print("Result : " + str(result) + "\n")