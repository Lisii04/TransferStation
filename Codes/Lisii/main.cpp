#include <iostream>
#include <math.h>
#include <numeric>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

bool IF_STOP = false;  // 是否停止
bool IF_SLOW = false;  // 是否减速
int zbera_count = 0;   // 识别到斑马线的帧数
int rhombus_count = 0; // 识别到菱形标的帧数

/* ROI区域提取
    @param inputFrame:输入图像
    @param points:ROI区域的边界坐标点集
*/
cv::Mat ROI_extract(cv::Mat inputFrame, std::vector<cv::Point> points)
{
    cv::Mat dst, src;
    src = inputFrame;
    cv::Mat ROI = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours; // 轮廓
    std::vector<cv::Point> pts;

    for (int i = 0; i < points.size(); i++)
    {
        pts.push_back(points[i]);
    }

    contours.push_back(pts);
    drawContours(ROI, contours, 0, cv::Scalar(255), -1); // 用白色填充多边形区域
    src.copyTo(dst, ROI);                                // 掩码运算

    return dst;
}

/* 两点间距离计算
    @param point0:起始点
    @param pointA:结束点
*/
double getDistance(cv::Point pointO, cv::Point pointA)
{
    double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);

    return distance;
}

/* 斑马线识别函数
    @param frame: 要处理的帧
    @param draw：要绘制轮廓的图像
*/
cv::Mat If_ZebraCrossing(cv::Mat frame, cv::Mat draw)
{

    /**** 识别前 图像处理 ****/
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;
    bool Is_approach = false;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point((max_X / 5) * 1, max_Y));           // LD
    points.push_back(cv::Point((max_X / 5) * 4, max_Y));           // RD
    points.push_back(cv::Point((max_X / 5) * 3, (max_Y / 5) * 3)); // RU
    points.push_back(cv::Point((max_X / 5) * 2, (max_Y / 5) * 3)); // RD
    ROI = ROI_extract(frame, points);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // 腐蚀处理
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(thres, erode, element);
    /**** 识别前图像处理结束 ****/

    /**** 识别图像轮廓 ****/
    // 存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(erode, contours, hierachy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE);
    // 多边形逼近的角点集合
    std::vector<std::vector<cv::Point2f>> all_points;

    int min_area = 100;
    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > min_area)
        {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            // 筛选出四边形
            if (points.size() == 4)
            {
                /**** 绘制图形 ****/
                // 绘制角点
                for (int i = 0; i < points.size(); i++)
                {
                    std::string text = std::to_string(i + 1);
                    cv::putText(draw, text, points[i], 1, 3, cv::Scalar(255, 0, 0), 2, 8);
                    cv::circle(draw, points[i], 4, cv::Scalar(255, 0, 0), -1, 8,
                               0);
                }

                // 绘制轮廓
                for (int j = 0; j < contours[i].size(); j++)
                {
                    cv::circle(draw, contours[i][j], 1, cv::Scalar(0, 255, 0), -1, 8,
                               0);
                }

                // 按长宽比例筛选四边形
                double len_to_wid_ratio;
                switch (getDistance(points[0], points[1]) >= getDistance(points[1], points[2]))
                {
                case 1:
                    len_to_wid_ratio = getDistance(points[0], points[1]) / getDistance(points[1], points[2]);
                    break;

                case 0:
                    len_to_wid_ratio = getDistance(points[1], points[2]) / getDistance(points[0], points[1]);
                    break;
                }
                if (len_to_wid_ratio < 5)
                {
                    cv::putText(draw, "##", points[0], 1, 3, cv::Scalar(255, 255, 0), 2, 8);
                    all_points.push_back(points);
                }
                /****    绘制图形轮廓结束    ****/
            }
        }
    }

    // 如果四边形数量大于4认为有可能识别到斑马线
    if (all_points.size() >= 4)
    {
        int x_min = 1000000, x_max = 0, y_min = 1000000, y_max = 0;

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
        // 斑马线边界点
        left_down_pt = cv::Point(x_min, y_max);
        left_up_pt = cv::Point(x_min, y_min);
        right_down_pt = cv::Point(x_max, y_max);
        right_up_pt = cv::Point(x_max, y_min);

        // 判断斑马线整体形状是否符合比例
        double zebra_ratio = 4.5;
        if (getDistance(left_down_pt, left_up_pt) * zebra_ratio < getDistance(left_down_pt, right_down_pt))
        {
            zbera_count++;
            std::string text = "ZebraCrossing ";
            text.append(std::to_string(frame.size().height - y_max));
            cv::putText(draw, text, left_up_pt, 1, 4, cv::Scalar(0, 0, 255), 4, 8);
            cv::line(draw, left_down_pt, left_up_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::line(draw, left_down_pt, right_down_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::line(draw, left_up_pt, right_up_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::line(draw, right_down_pt, right_up_pt, cv::Scalar(0, 0, 255), 4, 8, 0);
            if (frame.size().height - y_max < 150)
            {
                Is_approach = true;
            }
        }
    }

    // 判断是否停止
    cv::String text = "IF_STOP:";
    if ((zbera_count > 5) && Is_approach == true)
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

/* 菱形标识别函数
    @param frame: 要处理的帧
    @param draw: 要绘制轮廓的图像
    @param temp: 要匹配的模板图像
*/
cv::Mat If_Rhombus(cv::Mat frame, cv::Mat draw, cv::Mat temp)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point((max_X / 5) * 1.2, max_Y));             // LD
    points.push_back(cv::Point((max_X / 5) * 3.6, max_Y));             // RD
    points.push_back(cv::Point((max_X / 5) * 2.8, (max_Y / 5) * 3.5)); // RU
    points.push_back(cv::Point((max_X / 5) * 2.2, (max_Y / 5) * 3.5)); // LU
    ROI = ROI_extract(frame, points);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // 膨胀处理
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::dilate(thres, dilate, element);
    // Canny边缘检测
    cv::Canny(dilate, canny, 100, 200, 3);
    /* 识别前图像处理结束 */

    /**** 识别图像轮廓 ****/
    // 存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(canny, contours, hierachy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE);

    int min_area = 1;
    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > min_area)
        {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            // 筛选出四边形
            if (points.size() == 4)
            {
                // 对识别的四边形进行透视变换
                cv::Point2f src_points[4];
                cv::Point2f dst_points[4];

                int x_min = 1000000, x_max = 0, y_min = 1000000, y_max = 0;
                for (int i = 0; i < points.size(); i++)
                {
                    if (x_max < points[i].x)
                        x_max = points[i].x;
                    if (x_min > points[i].x)
                        x_min = points[i].x;
                    if (y_max < points[i].y)
                        y_max = points[i].y;
                    if (y_min > points[i].y)
                        y_min = points[i].y;
                }

                src_points[0] = cv::Point2f(x_min, y_min);
                src_points[1] = cv::Point2f(x_max, y_min);
                src_points[2] = cv::Point2f(x_min, y_max);
                src_points[3] = cv::Point2f(x_max, y_max);

                dst_points[0] = cv::Point2f(0.0, 0.0);
                dst_points[1] = cv::Point2f(100.0, 0.0);
                dst_points[2] = cv::Point2f(0.0, 100.0);
                dst_points[3] = cv::Point2f(100.0, 100.0);

                cv::Mat rotation, warp, warp_canny;
                rotation = getPerspectiveTransform(src_points, dst_points);
                cv::warpPerspective(dilate, warp, rotation, cv::Size(100, 100));

                // 透视变换得到warp进行匹配
                // 定义变量
                std::vector<cv::KeyPoint> keypoints1, keypoints2; // 定义检测的特征点存储容器
                cv::Mat descriptors1, descriptors2;               // 定义特征点描述信息为Mat类型

                // 创建sift特征检测器实例
                cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
                // 提取特征点
                detector->detect(temp, keypoints1, cv::noArray());
                detector->detect(warp, keypoints2, cv::Mat());

                // 获取特征点的特征向量
                detector->compute(temp, keypoints1, descriptors1);
                detector->compute(warp, keypoints2, descriptors2);

                // 定义匹配器的实例化 暴力匹配法
                cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

                // 进行暴力匹配
                std::vector<cv::DMatch> matches;

                matcher->match(descriptors1, descriptors2, matches);
                std::cout << matches.size() << std::endl;

                cv::drawKeypoints(draw, keypoints2, draw);
            }
        }
    }
    return draw;
}

void VideoProcess(cv::VideoCapture video)
{
    cv::namedWindow("a", 0);
    cv::resizeWindow("a", 1000, 500);

    cv::Mat temp, temp_canny;
    temp = cv::imread("1.png");

    int count = 0;
    while (1)
    {

        count++;

        cv::Mat frame, draw;
        video >> frame; // 从视频中读取一帧图像
        draw = frame;

        if (frame.empty())
            break; // 如果图像为空，表示已经读取完所有帧，跳出循环

        if (count > 10)
        {
            frame = If_ZebraCrossing(frame, draw);

            cv::imshow("a", frame);
            cv::waitKey();
        }

        // frame = If_Rhombus(temp, frame);

        // cv::imshow("a", frame);

        cv::waitKey(1);

        std::cout << "Processing frame " << count << std::endl;
    }
}

int main()
{
    cv::VideoCapture video;
    video.open("3.mp4"); // 打开视频文件/摄像头

    VideoProcess(video); // 处理视频帧

    video.release(); // 关闭视频文件/摄像头

    return 0;
}
