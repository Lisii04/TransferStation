#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <numeric>
#include <math.h>
#include <vector>

/* 绘制轮廓 */
void dfs(cv::Mat &drawer,
         const std::vector<std::vector<cv::Point>> &contours,
         const std::vector<cv::Vec4i> &hierachy,
         const int &id,
         const int &depth) {
    if (id == -1) return; // 如果轮廓 ID 为 -1，表示没有轮廓，直接返回
    static cv::Scalar COLOR_LIST[3] = { {220, 20, 20}, {20, 220, 20}, {20, 20, 220} };
    // 使用不同颜色绘制轮廓，以深度(depth)为索引选择颜色
    cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 1);
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0]) {
        dfs(drawer, contours, hierachy, i, depth + 1);  // 递归绘制子轮廓
    }
}

std::vector<cv::Mat> FrameProcess(cv::VideoCapture video)
{
    std::vector<cv::Mat> frames; // 存储处理后的帧图像
    int count = 0;
    while (1)
    {
        count++;
        cv::Mat frame;
        video >> frame; // 从视频中读取一帧图像

        if (frame.empty()) break; // 如果图像为空，表示已经读取完所有帧，跳出循环

        cv::Mat dst, dst1, dst2;

        cv::GaussianBlur(frame, dst, cv::Size(7, 7), 0, 0); // 对图像进行高斯模糊
        cv::threshold(dst, dst1, 100, 255, cv::THRESH_BINARY); // 阈值化处理
        cv::Canny(dst1, dst2, 200, 200, 3); // Canny边缘检测

        std::vector<std::vector<cv::Point>> contours; // 存储轮廓
        std::vector<cv::Vec4i> hierachy; // 存储轮廓层级信息
        cv::findContours(dst2, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE); // 查找轮廓

        for (int i = 0; i < contours.size(); i++)
        {
            int num = 0;
            double cenX = 0, cenY = 0, tempX = 0, tempY = 0;

            if (cv::contourArea(contours[i]) > 1000 && cv::contourArea(contours[i]) < 500000)
            {
                for (int j = 0; j < contours[i].size(); j++)
                {
                    if (cv::contourArea(contours[i]) > 200)
                        cv::circle(frame, contours[i][j], 2, cv::Scalar(0, 255, 0), -1, 8, 0); // 绘制轮廓
                    tempX += contours[i][j].x;
                    tempY += contours[i][j].y;
                    num++;
                }

                cenX = tempX / num;
                cenY = tempY / num;
                cv::Point center = cv::Point(cenX, cenY);

                if (cv::contourArea(contours[i]) > 1000 && cv::contourArea(contours[i]) < 500000)
                {
                    std::vector<cv::Point2f> points;
                    cv::approxPolyDP(contours[i], points, 10.0, true); // 多边形逼近

                    cv::circle(frame, center, 2, cv::Scalar(0, 255, 0), -1, 8, 0); // 绘制中心点

                    switch (points.size())
                    {
                    case 3:
                        cv::putText(frame, "Triangle", center, 1, 3, cv::Scalar(255, 0, 0), 1, 8, false); 
                        break;
                    case 4:
                        cv::putText(frame, "Rectangle", center, 1, 3, cv::Scalar(255, 0, 0), 1, 8, false); 
                        break;
                    default:
                        cv::putText(frame, "Round", center, 1, 3, cv::Scalar(255, 0, 0), 1, 8, false); 
                        break;
                    }

                    if (points.size() < 5)
                    {
                        for (int i = 0; i < points.size() - 1; i++) {
                            //cv::line(frame, points[i], points[i + 1], cv::Scalar(255, 0, 0), 5, 8, 0);   
                            cv::circle(frame, points[i], 5, cv::Scalar(255, 0, 0), -1, 8, 0); // 绘制角点
                        }
                    }
                }
            }
        }

        frames.push_back(frame); // 将处理后的帧图像添加到向量中
        std::cout << "Processing frame " << count << std::endl;
    }

    return frames; // 返回处理后的帧图像向量
}

int main()
{
    cv::VideoCapture video;
    video.open("123.mp4"); // 打开视频文件
    int count = 0;
    std::vector<cv::Mat> frames;

    frames = FrameProcess(video); // 处理视频帧

    video.release(); // 关闭视频文件

    cv::namedWindow("aaa", 0);
    cv::resizeWindow("aaa", 1600, 800);

    while (1)
    {
        count++;

        if (frames[count].empty()) break; // 如果帧为空，表示已经显示完所有帧，跳出循环
        cv::imshow("aaa", frames[count]); // 显示帧图像

        if (cv::waitKey(33) >= 0) break; // 等待33毫秒，如果按下键盘任意键，跳出循环
    }

    return 0;
}
