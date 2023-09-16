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


/* 帧处理函数 
        frame:要处理的帧
        drawMat：要绘制轮廓的图像
*/
cv::Mat FrameProcess(cv::Mat frame,cv::Mat drawMat)
{
    
    /***************  识别前 图像处理  ***************/
    cv::Mat dst, dst1, dst2,dst3;

    cv::GaussianBlur(frame, dst, cv::Size(7, 7), 0, 0); // 对图像进行高斯模糊

    cv::threshold(dst, dst1, 100, 255, cv::THRESH_BINARY); // 阈值化处理


    // cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(1, 1));
    // cv::erode(dst1, dst2, element);//腐蚀处理(无效，已去除)

    cv::Canny(dst1, dst3, 250, 250, 3); // Canny边缘检测

    /***************  识别前 图像处理结束  ***************/



    /***************  识别图像轮廓  ***************/

    std::vector<std::vector<cv::Point>> contours; // 存储轮廓
    std::vector<cv::Vec4i> hierachy; // 存储轮廓层级信息
    cv::findContours(dst3, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE); // 查找轮廓

    for (int i = 0; i < contours.size(); i++)
    {
        int num = 0;
        double cenX = 0, cenY = 0, tempX = 0, tempY = 0;

        if (cv::contourArea(contours[i]) > 1000 && cv::contourArea(contours[i]) < 500000)//按轮廓面积筛选
        {
            /**** 绘制图形轮廓并计算中心点 ****/
            for (int j = 0; j < contours[i].size(); j++)
            {
                cv::circle(drawMat, contours[i][j], 2, cv::Scalar(0, 255, 0), -1, 8, 0); // 绘制轮廓
                tempX += contours[i][j].x;
                tempY += contours[i][j].y;
                num++;
            }
            /****    绘制图形轮廓结束    ****/

            cenX = tempX / num;
            cenY = tempY / num;
            cv::Point center = cv::Point(cenX, cenY);

            /**** 不同形状分类 ****/
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true); // 多边形逼近

            cv::circle(drawMat, center, 2, cv::Scalar(0, 255, 0), -1, 8, 0); // 绘制中心点

            switch (points.size())//形状分类
            {
            case 3:
                cv::putText(drawMat, "Triangle", center, 1, 3, cv::Scalar(255, 0, 0), 3, 8, false); 
                break;
            case 4:
                cv::putText(drawMat, "Rectangle", center, 1, 3, cv::Scalar(255, 0, 0), 3, 8, false); 
                break;
            default:
                cv::putText(drawMat, "Round", center, 1, 3, cv::Scalar(255, 0, 0), 3, 8, false); 
                break;
            }
            /**** 不同形状分类结束 ****/

            /**** 绘制多边形顶点 ****/
            if (points.size() < 5)
            {
                for (int i = 0; i < points.size() - 1; i++) {
                    cv::circle(drawMat, points[i], 5, cv::Scalar(255, 0, 0), -1, 8, 0); // 绘制角点
                }
            }
            /**** 绘制多边形顶点结束 ****/
            
        
        }
    }

    return drawMat;
}


void VideoProcess(cv::VideoCapture video)
{
    cv::namedWindow("a", 0);
    cv::resizeWindow("a",1000,500);
    
    cv::namedWindow("b", 0);
    cv::resizeWindow("b",1000,500);

   
    int count = 0;
    while (1)
    {

        count++;
        cv::Mat frame;
        video >> frame; // 从视频中读取一帧图像
        if (frame.empty()) break; // 如果图像为空，表示已经读取完所有帧，跳出循环

        std::vector<cv::Mat> channels;
        split(frame, channels);
        cv::Mat blue, green, red;

        blue = channels.at(0);
        green = channels.at(1);
        red = channels.at(2);

        
        frame = FrameProcess(blue,frame);

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
