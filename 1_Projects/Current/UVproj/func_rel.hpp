#include <iostream>
#include <math.h>
#include <numeric>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <python3.10/Python.h>
#include <sys/time.h>
#include <stdlib.h>

/* ROI区域提取
    @param inputFrame 输入图像
    @param points ROI区域的边界坐标点集

    @return ROI区域的Mat矩阵
*/
cv::Mat ROI_extract(cv::Mat inputFrame, std::vector<cv::Point> points);

/* 两点间距离计算
    @param point0 起始点
    @param pointA 结束点

    @return distance 斜率
*/
double getDistance(cv::Point pointO, cv::Point pointA);

/* 绝对值计算
    @param input 输入值

    @return 输入值的绝对值
*/
template <typename T>
T getAbs(T input);

/* 直线斜率计算
    @param pointA 直线上第一点
    @param pointB 直线上第二点

    @return slope 斜率
*/
double getSlope(cv::Point pointA, cv::Point pointB);

/* 斑马线识别函数
    @param frame 要处理的帧
    @param draw 要绘制轮廓的图像
*/
int If_ZebraCrossing(cv::Mat frame);

/* 菱形标识别函数
    @param frame: 要处理的帧
    @param draw: 要绘制轮廓的图像
*/
int If_Rhombus(cv::Mat frame);

/* 车道线函数
    @param frame: 要处理的帧
*/
int LaneLine(cv::Mat frame);

/** 串口通信函数
 *
 */
int uart_send(int FLAG_TURN, int FLAG_SLOW, int FLAG_STOP, int DEVIATION);

// void VideoProcess(cv::VideoCapture video);