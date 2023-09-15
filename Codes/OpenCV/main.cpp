#include <opencv4/opencv2/opencv.hpp>
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

cv::Mat ImgProcess(cv::Mat src,int count)
{

    
    cv::Mat dst,dst1,dst2;

    cv::GaussianBlur(src,dst,cv::Size(7, 7), 0, 0);

    cv::threshold(dst,dst1,100,255,cv::THRESH_BINARY);



    
    cv::Canny(dst1, dst2,200,200,3);
    //cv::imshow("aaa",dst1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(dst2, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); i++)
    {
        int num = 0;
        double cenX = 0,cenY = 0,tempX = 0,tempY = 0;
        for (int j = 0; j < contours[i].size(); j++)
        {
            if(cv::contourArea(contours[i]) > 200)
            cv::circle(src,contours[i][j],3,cv::Scalar(0,255,0),-1,8,0);
            tempX += contours[i][j].x;
            tempY += contours[i][j].y;
            num ++;
        }
        cenX = tempX / num;
        cenY = tempY / num;
        cv::Point center = cv::Point(cenX,cenY);


        if(cv::contourArea(contours[i]) > 200)
        {     
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            switch (points.size())
            {
            case 3:
                cv::putText(src,"3",center,1,3,cv::Scalar(255,0,0),1,8,false);
                break;
            case 4:
                cv::putText(src,"4",center,1,3,cv::Scalar(255,0,0),1,8,false);
                break;
            default:
                cv::putText(src,"round",center,1,3,cv::Scalar(255,0,0),1,8,false);
                break;
            }


            // for (int i = 0;i < points.size()-1 ; i++) {
            //     //cv::line(src,points[i],points[i+1],cv::Scalar(255,0,0),5,8,0);   
            //     cv::circle(src,points[i],5,cv::Scalar(255,0,0),-1,8,0);
            // }
            
        }
        
       

    }
    
   
    std::cout << "frame" << count << std::endl;
    return src;
}


int main()
{

    cv::Mat src = cv::imread("ex1.png");
    if(src.empty())
    {
        std::cout << "open picture error!" << std::endl;
        return -1;
    }

    src = ImgProcess(src,1);
    cv::imshow("aaa",src);
    cv::waitKey();
    




    // cv::VideoCapture video;
    // video.open("123.mp4");
    // int count = 0;
    // std::vector<cv::Mat> frames;

    // while (1)
    // {
        
    //     count++;
    //     cv::Mat frame;
    //     video >> frame;

    //     if (frame.empty())break;
    //     frames.push_back(ImgProcess(frame,count));
    // }

    // video.release();



    // video.open("123.mp4");
    // cv::namedWindow("aaa",0);
    // cv::resizeWindow("aaa",600,1200);

    // count = 0;

    // while (1)
    // {
    //     count++;
    //     cv::imshow("aaa",frames[count]);
    //     if (frames[count].empty())break;
	// 	if (cv::waitKey(16) >= 0) break;
    // }
    

    // video.release();

        
    return 0;
}