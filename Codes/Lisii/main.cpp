#include <iostream>
#include <math.h>
#include <numeric>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

/* 绘制轮廓 */
void dfs(cv::Mat &drawer, const std::vector<std::vector<cv::Point>> &contours,
         const std::vector<cv::Vec4i> &hierachy, const int &id,
         const int &depth)
{
    if (id == -1)
        return; // 如果轮廓 ID 为 -1，表示没有轮廓，直接返回
    static cv::Scalar COLOR_LIST[3] = {
        {220, 20, 20}, {20, 220, 20}, {20, 20, 220}};
    // 使用不同颜色绘制轮廓，以深度(depth)为索引选择颜色
    cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 1);
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0])
    {
        dfs(drawer, contours, hierachy, i, depth + 1); // 递归绘制子轮廓
    }
}

cv::Mat ROI_extract(cv::Mat inputFrame)
{
    cv::Mat dst, src;
    src = inputFrame;
    cv::Mat ROI = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours; // 轮廓
    std::vector<cv::Point> pts;                   // 多边形角点集合

    int max_X = src.size().width;
    int max_Y = src.size().height;

    pts.push_back(cv::Point(0, max_Y));
    pts.push_back(cv::Point(max_X, max_Y));
    pts.push_back(cv::Point((max_X / 5) * 3, (max_Y / 2)));
    pts.push_back(cv::Point((max_X / 5) * 2, (max_Y / 2)));

    contours.push_back(pts);
    drawContours(ROI, contours, 0, cv::Scalar(255), -1); // 用白色填充多边形区域
    src.copyTo(dst, ROI);                                // 掩码运算

    // cv::namedWindow("dst", 0);
    // cv::resizeWindow("dst", 1000, 500);
    // imshow("dst", dst);

    return dst;
}

/* 帧处理函数
        frame:要处理的帧
        drawMat：要绘制轮廓的图像
*/
cv::Mat FrameProcess(cv::Mat frame, cv::Mat draw)
{

    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;

    ROI = ROI_extract(frame);

    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0); // 高斯滤波

    cv::threshold(bulr, thres, 100, 255, cv::THRESH_BINARY); // 阈值化处理

    cv::Canny(thres, canny, 100, 200, 3); // Canny边缘检测

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(canny, dilate, element);
    cv::erode(dilate, erode, element);

    /* 图像处理结束 */

    /***************  识别图像轮廓  ***************/

    std::vector<std::vector<cv::Point>> contours; // 存储轮廓
    std::vector<cv::Vec4i> hierachy;              // 存储轮廓层级信息
    cv::findContours(erode, contours, hierachy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE); // 查找轮廓

    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > 100)
        {
            /**** 绘制图形轮廓并计算中心点 ****/
            for (int j = 0; j < contours[i].size(); j++)
            {
                cv::circle(draw, contours[i][j], 1, cv::Scalar(0, 255, 0), -1, 8,
                           0); // 绘制轮廓
            }
            /****    绘制图形轮廓结束    ****/
        }
    }

    return draw;
}

void VideoProcess(cv::VideoCapture video)
{
    cv::namedWindow("a", 0);
    cv::resizeWindow("a", 1000, 500);

    // cv::namedWindow("b", 0);
    // cv::resizeWindow("b",1000,500);

    int count = 0;
    while (1)
    {

        count++;

        cv::Mat frame, temp;
        video >> frame; // 从视频中读取一帧图像
        temp = frame;

        if (frame.empty())
            break; // 如果图像为空，表示已经读取完所有帧，跳出循环

        if (count > 250)
        {
            frame = FrameProcess(temp, frame);
            // frame = ROI_extract(frame);

            cv::imshow("a", frame);
        }

        if (count > 300)
        {
            cv::waitKey();
        }
        cv::waitKey(1);

        std::cout << "Processing frame " << count << std::endl;
    }
}

int main()
{
    cv::VideoCapture video;
    video.open("123.mp4"); // 打开视频文件/摄像头

    VideoProcess(video); // 处理视频帧

    video.release(); // 关闭视频文件/摄像头

    return 0;
}
