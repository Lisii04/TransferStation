#include <iostream>
#include <math.h>
#include <numeric>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

bool IF_STOP = false;
int stop_count = 0;

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

double getDistance(cv::Point pointO, cv::Point pointA)

{
    double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);

    return distance;
}

/* 帧处理函数
    @param frame:要处理的帧
    @param draw：要绘制轮廓的图像
*/
cv::Mat FrameProcess(cv::Mat frame, cv::Mat draw)
{

    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;

    ROI = ROI_extract(frame);

    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0); // 高斯滤波

    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY); // 阈值化处理

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // cv::dilate(canny, dilate, element);
    cv::erode(thres, erode, element); // 腐蚀处理

    cv::Canny(erode, canny, 100, 200, 3); // Canny边缘检测

    /* 图像处理结束 */

    /***************  识别图像轮廓  ***************/

    std::vector<std::vector<cv::Point>> contours; // 存储轮廓
    std::vector<cv::Vec4i> hierachy;              // 存储轮廓层级信息
    cv::findContours(erode, contours, hierachy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE); // 查找轮廓

    std::vector<std::vector<cv::Point2f>> all_points;
    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > 100)
        {
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true); // 多边形逼近

            if (points.size() == 4)
            {
                /**** 绘制图形轮廓 ****/
                for (int i = 0; i < points.size(); i++)
                {
                    std::string text = std::to_string(i + 1);
                    cv::putText(draw, text, points[i], 1, 3, cv::Scalar(255, 0, 0), 2, 8);
                    cv::circle(draw, points[i], 4, cv::Scalar(255, 0, 0), -1, 8,
                               0); // 绘制角点
                }
                for (int j = 0; j < contours[i].size(); j++)
                {
                    cv::circle(draw, contours[i][j], 1, cv::Scalar(0, 255, 0), -1, 8,
                               0); // 绘制轮廓
                }

                switch (getDistance(points[0], points[1]) >= getDistance(points[1], points[2]))
                {
                case 1:
                    if ((getDistance(points[0], points[1]) / getDistance(points[1], points[2])) < 2.5)
                    {
                        all_points.push_back(points);
                    }

                    break;

                case 0:
                    if ((getDistance(points[1], points[2]) / getDistance(points[0], points[1])) < 5.0)
                    {
                        all_points.push_back(points);
                    }
                    break;
                }
                /****    绘制图形轮廓结束    ****/
            }
        }
    }

    if (all_points.size() >= 4)
    {
        int x_min = 1000000, x_max, y_min = 1000000, y_max;
        cv::Point left_up_pt, left_down_pt, right_up_pt, right_down_pt;
        for (int i = 0; i < all_points.size(); i++)
        {
            for (int j = 0; j < all_points[i].size(); j++)
            {
                if (x_max < all_points[i][j].x)
                    x_max = all_points[i][j].x;
                if (x_min > all_points[i][j].x)
                    x_min = all_points[i][j].x;
                if (y_max < all_points[i][j].y)
                    y_max = all_points[i][j].y;
                if (y_min > all_points[i][j].y)
                    y_min = all_points[i][j].y;
            }
        }
        left_down_pt = cv::Point(x_min, y_max);
        left_up_pt = cv::Point(x_min, y_min);
        right_down_pt = cv::Point(x_max, y_max);
        right_up_pt = cv::Point(x_max, y_min);

        if (getDistance(left_down_pt, left_up_pt) * 3 < getDistance(left_down_pt, right_down_pt))
        {
            stop_count++;

            cv::putText(draw, "ZebraCrossing", left_up_pt, 1, 4, cv::Scalar(0, 0, 255), 4, 8);
            cv::line(draw, left_down_pt, left_up_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::line(draw, left_down_pt, right_down_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::line(draw, left_up_pt, right_up_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::line(draw, right_down_pt, right_up_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
        }
    }

    cv::String text = "IF_STOP:";
    if (stop_count > 5)
    {
        IF_STOP = true;
        text.append("YES");
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
    }
    else
    {
        text.append("NO");
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
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

        // if (count > 250)
        // {
        //     frame = FrameProcess(temp, frame);

        //     cv::imshow("a", frame);
        // }

        // if (count > 300)
        // {
        //     cv::waitKey();
        // }

        frame = FrameProcess(temp, frame);

        cv::imshow("a", frame);

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
