import cv2
import matplotlib
import matplotlib.pyplot
import numpy
import NeuralNetwork_Use
from PIL import Image


# 绘制轮廓
def dfs(drawer, contours, hierarchy, id, depth):
    if id == -1:
        return

    COLOR_LIST = [(220, 20, 20), (20, 220, 20), (20, 20, 220)]
    cv2.drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 1)

    for i in range(hierarchy[id][2], -1, -1):
        dfs(drawer, contours, hierarchy, i, depth + 1)


# 帧处理函数
#     frame:要处理的帧
#     drawMat：要绘制轮廓的图像
def frame_process(frame, draw_mat, color):
    # 识别前图像处理
    dst = cv2.GaussianBlur(frame, (7, 7), 0, 0)
    _, dst1 = cv2.threshold(dst, 100, 255, cv2.THRESH_BINARY)
    dst3 = cv2.Canny(dst1, 300, 300, 3)

    # 识别图像轮廓
    contours, hierarchy = cv2.findContours(dst3, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for i in range(len(contours)):
        num = 0
        cenX = 0
        cenY = 0
        tempX = 0
        tempY = 0

        maxX = 0
        maxY = 0
        minX = 100000
        minY = 100000

        if 3500 < cv2.contourArea(contours[i]) < 500000:
            for j in range(len(contours[i])):
                tempX += contours[i][j][0][0]
                tempY += contours[i][j][0][1]

                if minX > contours[i][j][0][0]:
                    minX = contours[i][j][0][0]
                if minY > contours[i][j][0][1]:
                    minY = contours[i][j][0][1]
                if maxX < contours[i][j][0][0]:
                    maxX = contours[i][j][0][0]
                if maxY < contours[i][j][0][1]:
                    maxY = contours[i][j][0][1]


                num += 1

            cenX = tempX / num
            cenY = tempY / num
            center = (int(cenX), int(cenY))

            crop = dst1[int(minY):int(maxY), int(minX):int(maxX)]

            cv2.imshow("a", crop)
            cv2.waitKey(1)
            # cv2.imwrite(str(i) + ".png", crop)

            image = Image.fromarray(crop)

            small_img = image.resize((28, 28))

            img_array = numpy.array(small_img.convert('L'))

            img_array = 255 - img_array

            matplotlib.pyplot.imshow(img_array, cmap='Greys', interpolation='None')
            # matplotlib.pyplot.waitforbuttonpress()

            fl = open("test.csv", "a+")

            points = cv2.approxPolyDP(contours[i], 10.0, True)

            if len(points) == 3:
                fl.write("1")

                img_data = numpy.reshape(img_array, 784)

                for i in img_data:
                    fl.write("," + str(img_data[i]))

                fl.write("\n")
            elif len(points) == 4:
                fl.write("2")

                img_data = numpy.reshape(img_array, 784)

                for i in img_data:
                    fl.write("," + str(img_data[i]))

                fl.write("\n")
            else:
                fl.write("0")

                img_data = numpy.reshape(img_array, 784)

                for i in img_data:
                    fl.write("," + str(img_data[i]))

                fl.write("\n")

    return draw_mat


def video_process(video_path):
    video = cv2.VideoCapture(video_path)
    cv2.namedWindow("a", 0)
    cv2.resizeWindow("a", 1000, 500)

    count = 0
    while True:
        count += 1
        ret, frame = video.read()
        if not ret:
            break

        Hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        Yellow_low = numpy.array([10, 43, 46])
        Yellow_high = numpy.array([34, 255, 255])

        Red_low_1 = numpy.array([0, 43, 46])
        Red_high_1 = numpy.array([10, 255, 255])

        Red_low_2 = numpy.array([156, 43, 46])
        Red_high_2 = numpy.array([180, 255, 255])

        Yellow_mask = cv2.inRange(Hsv, Yellow_low, Yellow_high)
        Red_mask_1 = cv2.inRange(Hsv, Red_low_1, Red_high_1)
        Red_mask_2 = cv2.inRange(Hsv, Red_low_2, Red_high_2)

        Red_mask = cv2.bitwise_or(Red_mask_1, Red_mask_2)

        frame = frame_process(Red_mask, frame, "RED")
        frame = frame_process(Yellow_mask, frame, "YELLOW")

        # cv2.imshow("a", frame)
        cv2.waitKey(1)

        print(f"Processing frame {count}")

    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    video_path = "2.mp4"
    video_process(video_path)
