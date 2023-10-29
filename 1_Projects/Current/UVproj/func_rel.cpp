#include "func_rel.hpp"

bool FLAG_STOP = false; // 是否停止
bool FLAG_SLOW = false; // 是否减速

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
double getAbs(double input)
{
    return ((input > 0) ? (input) : (-input));
}
int getAbs(int input)
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
    if (x_abs == 0.0)
    {
        x_abs += 0.0001;
    }

    double slope = y_abs / x_abs;
    return slope;
}

/* 已知斜率和直线上一点求截距
 *
 */
double getIntercept(double slope, cv::Point point)
{
    return (point.y - point.x * slope);
}

/* 斑马线识别函数
    @param frame 要处理的帧
    @param draw 要绘制轮廓的图像
*/
int If_ZebraCrossing(cv::Mat frame)
{

    /**** 识别前 图像处理 ****/
    cv::Mat gray, ROI, bulr, thres, canny, element, dilate;
    bool Is_approach = false;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (0.01 / 5.0), max_Y * (4.95 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (4.99 / 5.0), max_Y * (4.95 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (4.85 / 5.0), max_Y * (3.0 / 5.0)));  // RU
    points.push_back(cv::Point(max_X * (0.15 / 5.0), max_Y * (3.0 / 5.0)));  // LU
    ROI = ROI_extract(frame, points);
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
                }
            }
        }
    }
    /**** 识别斑马线块 结束 ****/

    /**** 识别斑马线 ****/
    // 如果四边形数量大于3认为有可能识别到斑马线
    if (all_points.size() >= 12)
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
        // 求上边斜率
        double up_slope = 0;
        if (getDistance(rect[0], rect[1]) > getDistance(rect[1], rect[2]))
        {
            up_slope = getAbs(getSlope(rect[0], rect[1]));
        }
        else
        {
            up_slope = getAbs(getSlope(rect[1], rect[2]));
        }
        // 求最靠近视频流底部的点
        double y_max = 0;
        for (int i = 0; i < 4; i++)
        {
            if (y_max < rect[i].y)
                y_max = rect[i].y;
        }

        // 定义斑马线总长宽比
        double zebra_ratio = 3;
        // 求置信度(凸包面积 + 补偿面积 ?= 斑马线块总面积*2.3)
        double confidence_level = ((hull_area + white_extern_area) <= (white_area * 2.3)) ? ((hull_area + white_extern_area) / (white_area * 2.5)) : ((white_area * 2.5) / (hull_area + white_extern_area));
        /* 最后的筛选：
            1.判断外接矩形长宽是否符合比例
            2.判断置信度是否大于0.8
            3.判断斑马线距离视频流底部距离是否小于500px
           如果满足：
            识别到斑马线的帧数（zbera_count）+1
        */
        if ((length * zebra_ratio < width) && (confidence_level >= 0.8) && (frame.size().height - y_max < 1000) && (up_slope <= 0.05))
        {
            zbera_count++;
            Is_approach = true;
        }
    }
    /**** 识别斑马线 结束 ****/

    /* 判断是否停止：
        当 zbera_count > 5 且 斑马线距离视频流底部距离满足条件 时 输出停止信号
    */
    if ((zbera_count > 10) && Is_approach == true)
    {
        FLAG_STOP = true;
    }

    return 1;
}

/* 菱形标识别函数
    @param frame: 要处理的帧
    @param draw: 要绘制轮廓的图像
*/
int If_Rhombus(cv::Mat frame)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (1.0 / 5.0), max_Y * (4.95 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (4.0 / 5.0), max_Y * (4.95 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (3.0 / 5.0), max_Y * (3.0 / 5.0)));  // RU
    points.push_back(cv::Point(max_X * (2.0 / 5.0), max_Y * (3.0 / 5.0)));  // LU
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

    // 定义-存储满足条件的四边形角点集合
    std::vector<std::vector<cv::Point2f>> all_point_sets;

    int min_area = 2000;
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
                    }
                }
            }
        }
    }

    /* 判断是否减速：
        当 rhombus_count > 5时 输出停止信号
    */
    if (rhombus_count > 15)
    {
        FLAG_SLOW = true;
    }

    return 1;
}

/* 车道线函数
    @param frame: 要处理的帧
*/
int LaneLine(cv::Mat frame)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element_erode, element_dilate, dilate;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (0.0 / 5.0), max_Y * (5.0 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (5.0 / 5.0), max_Y * (5.0 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (5.0 / 5.0), max_Y * (3.0 / 5.0))); // RU
    points.push_back(cv::Point(max_X * (0.0 / 5.0), max_Y * (3.0 / 5.0))); // LU
    ROI = ROI_extract(frame, points);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 170, 255, cv::THRESH_BINARY);

    cv::Canny(thres, canny, 100, 200, 3);
    /* 识别前图像处理结束 */

    std::vector<cv::Vec4f> lines;
    // 创建一个包含直线斜率和截距的特征向量的数据集
    std::vector<std::vector<cv::Point2d>> features;
    std::vector<cv::Point2d> left;
    std::vector<cv::Point2d> right;
    features.push_back(left);
    features.push_back(right);
    cv::HoughLinesP(canny, lines, 1., CV_PI / 180, 100, 50, 30);
    for (int i = 0; i < lines.size(); ++i)
    {
        cv::Vec4i line_ = lines[i];
        cv::Point pointA = cv::Point(line_[0], line_[1]), pointB = cv::Point(line_[2], line_[3]);
        double slope = getSlope(pointA, pointB);
        double intercept = getIntercept(slope, pointA);
        if (getAbs(slope) >= 0.5 && getAbs(slope) < 500000)
        {
            if (slope > 0)
            {
                features[0].push_back(cv::Point2d(slope, intercept));
            }
            else
            {
                features[1].push_back(cv::Point2d(slope, intercept));
            }
        }
    }

    if (features[0].size() != 0 && features[1].size() != 0)
    {
        std::vector<cv::Point2d> lanelines;
        for (int i = 0; i < features.size(); i++)
        {
            double slope = 0, intercept = 0;

            for (int j = 0; j < features[i].size(); j++)
            {
                slope += features[i][j].x;
                intercept += features[i][j].y;
            }

            lanelines.push_back(cv::Point2d((slope / features[i].size()), (intercept / features[i].size())));
        }
        std::vector<int> center;
        for (int x = 0; x < max_X; x++)
        {
            if (getAbs((x * lanelines[0].x + lanelines[0].y) - (x * lanelines[1].x + lanelines[1].y)) <= 10)
            {
                center.push_back(x);
            }
        }
        double center_x = 0;
        for (int i = 0; i < center.size(); i++)
        {
            center_x += center[i];
        }
        center_x = center_x / center.size();

        int Deviation = (int)((double)center_x - (double)max_X / 2.0);

        return Deviation;
    }
    else
    {
        return 0;
    }
}

/** 串口通信函数
 *  @param DEVIATION
 *  @param TURN_DIRECTION (0不转弯  1左转  2右转)
 *  @param FLAG_SLOW (0不减速 1减速)
 *  @param FLAG_STOP (0不停止 1停止)
 *  @retval 1 或 0
 */
int uart_send(int DEVIATION, int TURN_DIRECTION, int FLAG_SLOW, int FLAG_STOP)
{
    // 初始化python接口
    Py_Initialize();
    if (!Py_IsInitialized())
    {
        std::cout << "python init fail" << std::endl;
        return 0;
    }
    // 初始化使用的变量
    PyObject *pModule = NULL;
    PyObject *pFunc = NULL;
    PyObject *pName = NULL;

    // 初始化python系统文件路径，保证可以访问到 .py文件
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('..')");

    // 调用python文件名。
    pModule = PyImport_ImportModule("uart");

    // 调用函数
    pFunc = PyObject_GetAttrString(pModule, "uart_send");

    // 给python传参数
    PyObject *pArgs = PyTuple_New(4);

    // 0：第一个参数，传入 float 类型的值 2.242
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("i", DEVIATION));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", TURN_DIRECTION));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("i", FLAG_SLOW));
    PyTuple_SetItem(pArgs, 3, Py_BuildValue("i", FLAG_STOP));

    // 使用C++的python接口调用该函数
    PyObject *pReturn = PyEval_CallObject(pFunc, pArgs);

    // 结束python接口初始化
    Py_Finalize();
    return 1;
}

// void VideoProcess(cv::VideoCapture video)
// {
//     cv::namedWindow("a", 0);
//     cv::resizeWindow("a", 1000, 500);

//     // FPS-------------------------------------------
//     float time_use = 0;
//     float all_time_use = 0;
//     struct timeval all_start;
//     struct timeval all_end;

//     struct timeval start;
//     struct timeval end;
//     gettimeofday(&all_start, NULL);
//     // ----------------------------------------------

//     int count = 0;
//     while (1)
//     {
//         gettimeofday(&start, NULL);

//         count++;

//         cv::Mat frame;
//         video >> frame; // 从视频中读取一帧图像
//         if (frame.empty())
//         {
//             gettimeofday(&all_end, NULL);
//             all_time_use = ((all_end.tv_sec - all_start.tv_sec) * 1000000 + (all_end.tv_usec - all_start.tv_usec));
//             std::cout << "Average FPS:" << (((float)count) / (all_time_use / 1000000)) << std::endl;
//             break; // 如果图像为空，表示已经读取完所有帧，跳出循环
//         }

//         int DEVIATION = 0;

//         if (count > 0)
//         {
//             if (FLAG_SLOW == 0)
//                 If_Rhombus(frame);
//             else
//                 If_ZebraCrossing(frame);
//             DEVIATION = LaneLine(frame);

//             uart_send(0, FLAG_SLOW, FLAG_STOP, DEVIATION);
//         }

//         // FPS--------------------------------------------------------------------------
//         gettimeofday(&end, NULL);
//         time_use = ((end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec));
//         std::cout << "Processing frame " << count << "|FPS:" << 1.0 / (time_use / 1000000) << "|DEV:" << DEVIATION << std::endl;
//         //-------------------------------------------------------------------------
//     }
// }
