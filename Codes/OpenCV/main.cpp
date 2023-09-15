#include <opencv2/opencv.hpp>
#include <iostream>
#include <numeric>
#include <math.h>
#include <vector>

/***** 求两点间距离*****/
float getDistance(cv::Point pointO, cv::Point pointA)
{
    float distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}


void dfs(cv::Mat &drawer,
         const std::vector< std::vector<cv::Point> > &contours,
         const std::vector< cv::Vec4i > &hierachy,
         const int &id,
         const int &depth) {
    if (id == -1) return;
    static cv::Scalar COLOR_LIST[3] = { {220, 20, 20}, {20, 220, 20}, {20, 20, 220} };
    cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 1);
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0]) {
        dfs(drawer, contours, hierachy, i, depth + 1);  // 向内部的子轮廓递归
    }
}

void ImgProcess(cv::Mat src)
{

    
    cv::Mat dst,dst1;
    
    cv::Canny(src, dst1,200,200,3);
    //cv::imshow("aaa",dst1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(dst1, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point2f> points;
        cv::approxPolyDP(contours[i], points, 10.0, true);

        
        for (int i = 0;i < contours.size() ; i++) {
            std::vector<cv::Vec4f> lines;
            cv::HoughLines(contours[i],lines,1,1,10,40,20);
            std::cout << lines.size() << std::endl;
        }

        // cv::Mat temp;
        // for (int i = 0;i < points.size()-1 ; i++) {
        //     cv::line(temp,points[i],points[i+1],cv::Scalar(255,0,0),3,8,0);    
        // }


        // temp.release();
    }
    
   

    //cv::imshow("aaa",src);
}


int main()
{

    // cv::Mat src = cv::imread("/home/lisii/Documents/Github_repos/TransferStation/Codes/OpenCV/build/ex1.png");
    // if(src.empty())
    // {
    //     std::cout << "open picture error!" << std::endl;
    //     return -1;
    // }

    // ImgProcess(src);
    // cv::waitKey();
    

    cv::VideoCapture video;
    video.open("ex.mp4");
    cv::namedWindow("aaa",0);
    cv::resizeWindow("aaa",400,800);

    while (1)
    {
        cv::Mat frame;
        video >> frame;
        ImgProcess(frame);
        if (frame.empty())break;
		if (cv::waitKey(16) >= 0) break;
    }
    

    video.release();

        
    return 0;
}