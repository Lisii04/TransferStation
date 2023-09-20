import numpy
import matplotlib.pyplot
import scipy.special
from PIL import Image
from tqdm import tqdm
import tkinter as tk
import tkinter.filedialog

def selectPath():
    path_ = tk.filedialog.askopenfilename()

    path_ = path_.replace("\\", "/")

    return path_

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

print(">Select train datas :[Enter]")

input()

TRAIN_FILE_PATH = selectPath()

print("\033[0;32m" + ">>Train datas selected in :" + TRAIN_FILE_PATH + "\033[0m" + "\n")

print(">Select test datas :[Enter]")

input()

TEST_FILE_PATH = selectPath()

print("\033[0;32m" + ">>Test datas selected in :" + TEST_FILE_PATH + "\033[0m" + "\n")

# INIT
rows = inputnodes
columns = hiddennodes

n = NeuralNetwork(inputnodes, hiddennodes, outputnodes, learningrate)

# TRAIN

print('>Opening train data...')

data = numpy.loadtxt(open(TRAIN_FILE_PATH, "rb"), delimiter=",")

targetlist = numpy.loadtxt(open(TRAIN_FILE_PATH, "rb"), delimiter=",", usecols=[0])

length = len(targetlist)
output_nodes = 10
i = 0

print("\033[0;32m" + '>>Train data opened in : ' + TRAIN_FILE_PATH + "\033[0m" + "\n")
print('>Training process : ')
for record1 in tqdm(data):
    inputs = (numpy.asfarray(data[i][1:]) / 255.0 * 0.99) + 0.01

    targets = numpy.zeros(output_nodes) + 0.01

    targets[int(targetlist[i])] = 0.99

    i += 1

    n.train(inputs, targets)

    pass

numpy.save('wih.npy', n.get_wih())

print("\033[0;32m" + ">Wih data saved in : wih.npy" + "\033[0m")

numpy.save('who.npy', n.get_who())

print("\033[0;32m" + ">Who data saved in : who.npy" + "\033[0m")

# TEST
tests = numpy.loadtxt(open(TEST_FILE_PATH, "rb"), delimiter=",")

testTargetlist = numpy.loadtxt(open(TEST_FILE_PATH, "rb"), delimiter=",", usecols=[0])

length = len(testTargetlist)
i = 0
score = 0

print('>Test data opened in : ' + TEST_FILE_PATH)
print('>Test process : ')
for record2 in tqdm(tests):
    result = n.query(numpy.asfarray(tests[i][1:]))
    correct = testTargetlist[i]

    label = numpy.argmax(result)
    
    if label == correct:
        score += 1
    else:
        pass
    i += 1

print("Score : " + str(score / length))
