#include "func.hpp"

bool IF_READY = false;
bool IF_STOP = false; // 是否停止
bool IF_SLOW = false; // 是否减速

int ready_count = 0;
int zbera_count = 0;   // 识别到斑马线的帧数
int rhombus_count = 0; // 识别到菱形标的帧数

/* ROI区域提取
    @param inputFrame 输入图像
    @param points ROI区域的边界坐标点集

    @return ROI区域的Mat矩阵
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
    @param point0 起始点
    @param pointA 结束点

    @return distance 斜率
*/
double getDistance(cv::Point pointO, cv::Point pointA)
{
    double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);

    return distance;
}

/* 绝对值计算
    @param input 输入值

    @return 输入值的绝对值
*/
template <typename T>
T getAbs(T input)
{
    return ((input > 0) ? (input) : (-input));
}

/* 直线斜率计算
    @param pointA 直线上第一点
    @param pointB 直线上第二点

    @return slope 斜率
*/
double getSlope(cv::Point pointA, cv::Point pointB)
{
    double y_abs = pointA.y - pointB.y;
    double x_abs = pointA.x - pointB.x;
    double slope = y_abs / x_abs;
    return slope;
}

/* 斑马线识别函数
    @param frame 要处理的帧
    @param draw 要绘制轮廓的图像
*/
cv::Mat If_ZebraCrossing(cv::Mat frame, cv::Mat draw)
{

    /**** 识别前 图像处理 ****/
    cv::Mat gray, ROI, bulr, thres, canny, element, dilate;
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
    line(draw, points[1], points[2], cv::Scalar(0, 255, 0), 1, 8);
    line(draw, points[2], points[3], cv::Scalar(0, 255, 0), 1, 8);
    line(draw, points[3], points[0], cv::Scalar(0, 255, 0), 1, 8);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // 腐蚀处理(防止斑马线与停止线粘连)
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(thres, dilate, element);
    /**** 识别前图像处理 结束 ****/

    /**** 识别斑马线块 ****/
    // 定义-存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 定义-存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(dilate, contours, hierachy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE);
    // 定义-存储满足条件的四边形角点集合
    std::vector<cv::Point> all_points;

    // 定义-斑马线白块的总面积
    double white_area = 0;
    // 定义-斑马线白块的补偿面积(应用: （凸包面积 + 补偿面积） 应等于 （斑马线白块的总面积 * 2） )
    double white_extern_area = 0;

    // 定义-最小轮廓面积
    int min_area = 1000;
    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > min_area)
        {
            // 定义-存储多边形逼近的角点
            std::vector<cv::Point2f> points;
            // 多边形逼近
            cv::approxPolyDP(contours[i], points, 10.0, true);

            // 筛选出四边形
            if (points.size() == 4)
            {

                // 绘制角点
                for (int j = 0; j < points.size(); j++)
                {
                    std::string text = std::to_string(j + 1);
                    cv::putText(draw, text, points[j], 1, 3, cv::Scalar(255, 0, 0), 2, 8);
                    cv::circle(draw, points[j], 4, cv::Scalar(255, 0, 0), -1, 8,
                               0);
                }
                // 绘制轮廓
                for (int j = 0; j < contours[i].size(); j++)
                {
                    cv::circle(draw, contours[i][j], 1, cv::Scalar(0, 255, 0), -1, 8,
                               0);
                }

                // 按长宽比例筛选四边形

                // 定义-长宽比(利用问号表达式使其大于一)
                double len_to_wid_ratio = (getDistance(points[0], points[1]) >= getDistance(points[1], points[2])) ? (getDistance(points[0], points[1]) / getDistance(points[1], points[2])) : (getDistance(points[1], points[2]) / getDistance(points[0], points[1]));

                if (len_to_wid_ratio < 6)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        all_points.push_back(points[j]);
                    }
                    if (white_extern_area == 0)
                    {
                        white_extern_area = cv::contourArea(contours[i]);
                    }
                    white_area += cv::contourArea(contours[i]);
                    cv::putText(draw, "ZebraBlock", points[0], 1, 1, cv::Scalar(255, 255, 0), 2, 8);
                }
            }
        }
    }
    /**** 识别斑马线块 结束 ****/

    /**** 识别斑马线 ****/
    // 如果四边形数量大于3认为有可能识别到斑马线
    if (all_points.size() >= 16)
    {
        // 定义-凸包点集
        std::vector<cv::Point> hull_points;
        // 求凸包点集
        cv::convexHull(all_points, hull_points);
        // 求凸包面积
        double hull_area = cv::contourArea(hull_points);

        // 定义-最小外接矩形
        cv::RotatedRect box;
        cv::Point2f rect[4];
        // 计算每个轮廓最小外接矩形
        box = cv::minAreaRect(cv::Mat(all_points));
        // 把最小外接矩形四个端点复制给rect数组
        box.points(rect);
        // 求外接矩形长宽
        unsigned int width = (getDistance(rect[0], rect[1]) > getDistance(rect[1], rect[2])) ? (getDistance(rect[0], rect[1])) : (getDistance(rect[1], rect[2])),
                     length = (getDistance(rect[0], rect[1]) < getDistance(rect[1], rect[2])) ? (getDistance(rect[0], rect[1])) : (getDistance(rect[1], rect[2]));

        // 求最靠近视频流底部的点
        double y_max = 0;
        for (int i = 0; i < 4; i++)
        {
            if (y_max < rect[i].y)
                y_max = rect[i].y;
        }

        // 定义斑马线总长宽比
        double zebra_ratio = 4.5;
        // 求置信度(凸包面积 + 补偿面积 ?= 斑马线块总面积*2)
        double confidence_level = ((hull_area + white_extern_area) <= (white_area * 2)) ? ((hull_area + white_extern_area) / (white_area * 2)) : ((white_area * 2) / (hull_area + white_extern_area));
        // 文字信息
        std::string text = "ZebraCrossing|Score:";
        text.append(std::to_string(confidence_level));
        text.append("|Dist:");
        text.append(std::to_string(frame.size().height - y_max));

        /* 最后的筛选：
            1.判断外接矩形长宽是否符合比例
            2.判断置信度是否大于0.8
            3.判断斑马线距离视频流底部距离是否小于100px
           如果满足：
            识别到斑马线的帧数（zbera_count）+1
        */
        if ((length * zebra_ratio < width) && (confidence_level >= 0.8) && (frame.size().height - y_max < 100))
        {
            // 视觉显示
            cv::putText(draw, text, rect[0], 1, 3, cv::Scalar(0, 255, 0), 2, 8);
            for (int j = 0; j < 4; j++)
            {
                line(draw, rect[j], rect[(j + 1) % 4], cv::Scalar(0, 255, 0), 3, 8); // 绘制最小外接矩形每条边
            }
            //

            zbera_count++;
            Is_approach = true;
        }
        else
        {
            cv::putText(draw, text, rect[0], 1, 3, cv::Scalar(0, 0, 255), 2, 8);
            for (int j = 0; j < 4; j++)
            {
                line(draw, rect[j], rect[(j + 1) % 4], cv::Scalar(0, 0, 255), 3, 8); // 绘制最小外接矩形每条边
            }
        }
    }
    /**** 识别斑马线 结束 ****/

    /* 判断是否停止：
        当 zbera_count > 5 且 斑马线距离视频流底部距离满足条件 时 输出停止信号
    */
    cv::String text = "IF_STOP:";
    if ((zbera_count > 5) && Is_approach == true)
    {
        IF_STOP = true;
    }

    if (IF_STOP == true)
    {
        text.append("YES");
        cv::putText(draw, std::to_string(zbera_count), cv::Point(50, 100), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
    }
    else
    {
        text.append("NO");
        cv::putText(draw, std::to_string(zbera_count), cv::Point(50, 100), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
    }

    return draw;
}

/* 菱形标识别函数
    @param frame: 要处理的帧
    @param draw: 要绘制轮廓的图像
*/
cv::Mat If_Rhombus(cv::Mat frame, cv::Mat draw)
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
    line(draw, points[1], points[2], cv::Scalar(0, 255, 0), 1, 8);
    line(draw, points[2], points[3], cv::Scalar(0, 255, 0), 1, 8);
    line(draw, points[3], points[0], cv::Scalar(0, 255, 0), 1, 8);
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
    cv::findContours(canny, contours, hierachy, cv::RETR_LIST,
                     cv::CHAIN_APPROX_NONE);

    // 定义-存储满足条件的四边形角点集合
    std::vector<std::vector<cv::Point2f>> all_point_sets;

    int min_area = 5000;
    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > min_area)
        {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            bool If_coincide = false;
            // 添加筛选条件防止点重合
            for (int i = 0; i < points.size(); i++)
            {
                if (getDistance(points[i], points[(i + 1) % 4]) <= 25)
                    If_coincide = true;
            }

            if (If_coincide == true)
                continue;

            // 筛选出四边形
            if (points.size() == 4)
            {
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

                // 定义-长宽比(利用问号表达式使其大于一)
                double len_to_wid_ratio = (getDistance(points[0], points[1]) >= getDistance(points[1], points[2])) ? (getDistance(points[0], points[1]) / getDistance(points[1], points[2])) : (getDistance(points[1], points[2]) / getDistance(points[0], points[1]));

                if (len_to_wid_ratio < 2)
                {
                    double diag_slope_1, diag_slope_2;
                    if ((getAbs(getSlope(points[0], points[2])) < 1) && (getAbs(getSlope(points[1], points[3])) > 1))
                    {
                        diag_slope_1 = getAbs(getSlope(points[0], points[2])),
                        diag_slope_2 = getAbs(getSlope(points[1], points[3]));
                    }
                    else if ((getAbs(getSlope(points[0], points[2])) > 1) && (getAbs(getSlope(points[1], points[3])) < 1))
                    {
                        diag_slope_2 = getAbs(getSlope(points[0], points[2])),
                        diag_slope_1 = getAbs(getSlope(points[1], points[3]));
                    }
                    else
                    {
                        continue;
                    }

                    double product = diag_slope_1 * diag_slope_2;

                    if ((diag_slope_1 < 0.5) && (diag_slope_2 > 2.5))
                    {
                        rhombus_count++;
                        std::string result;
                        result.append(std::to_string(diag_slope_1));
                        result.append("|");
                        result.append(std::to_string(diag_slope_2));
                        result.append("|");
                        result.append(std::to_string(product));
                        all_point_sets.push_back(points);
                        cv::putText(draw, result, points[0], 1, 3, cv::Scalar(255, 255, 0), 2, 8);
                        cv::putText(draw, std::to_string(cv::contourArea(contours[i])), cv::Point(points[0].x, points[0].y - 50), 1, 3, cv::Scalar(255, 0, 255), 2, 8);
                    }
                }
            }
        }
    }

    /* 判断是否减速：
        当 rhombus_count > 5时 输出停止信号
    */
    cv::String text = "IF_SLOW:";
    if (rhombus_count > 3)
    {
        IF_SLOW = true;
    }

    if (IF_SLOW == true)
    {
        text.append("YES");
        cv::putText(draw, std::to_string(rhombus_count), cv::Point(50, 100), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
    }
    else
    {
        text.append("NO");
        cv::putText(draw, std::to_string(rhombus_count), cv::Point(50, 100), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
    }

    return draw;
}

void VideoProcess(cv::VideoCapture video)
{
    cv::namedWindow("a", 0);
    cv::resizeWindow("a", 1000, 500);

    int count = 0;
    while (1)
    {

        count++;

        cv::Mat frame, draw;
        video >> frame; // 从视频中读取一帧图像
        draw = frame;

        if (frame.empty())
            break; // 如果图像为空，表示已经读取完所有帧，跳出循环

        if (count > 100)
        {
            frame = If_ZebraCrossing(frame, draw);

            cv::imshow("a", frame);
            cv::waitKey(1);
        }

        cv::waitKey(1);

        std::cout << "Processing frame " << count << std::endl;
    }
}

/* 启动函数
    @param frame: 要处理的帧
*/
cv::Mat UVstart(cv::Mat frame)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // Canny边缘检测
    cv::Canny(dilate, canny, 100, 200, 3);
    /* 识别前图像处理结束 */

    /**** 识别图像轮廓 ****/
    // 存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(canny, contours, hierachy, cv::RETR_LIST,
                     cv::CHAIN_APPROX_NONE);

    bool If_hexagon = false;

    if (IF_READY == false)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            // 筛选出六边形
            if (points.size() == 6)
            {
                If_hexagon = true;
            }
        }
    }
    else
    {
        for (int i = 0; i < contours.size(); i++)
        {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            // 筛选出六边形
            if (points.size() != 6)
            {
                ready_count++;
            }
        }
    }

    if (If_hexagon == true && contours.size() == 1)
    {
        IF_READY = true;
    }
}
