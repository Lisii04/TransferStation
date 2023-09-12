#include <opencv2/opencv.hpp>
#include <iostream>
#include <numeric>
#include <math.h>
#include <vector>
#include <unistd.h>
 
using namespace cv;
using namespace std;
 
 
double VectorVar(const vector<double> &A)
{
	double sum = accumulate(begin(A),end(A), 0.0);
    double mean =  sum / A.size();
    
	double variance  = 0.0;
    for (uint16_t i = 0 ; i < A.size() ; i++)
    {
        variance = variance + pow(A[i]-mean,2);
    }
    variance = variance/A.size();
    return variance;
}

int main(int argc,char** argv)
{
    VideoCapture video;
    video.open("ex.mp4");
    Mat frame;
    namedWindow("video-demo", 0);
    resizeWindow("video-demo",800,500);
    while(1)
    {
        
        video >> frame;
        if (frame.empty())break;

        Mat src = frame;
        //Mat src = imread("ex2.png");
        if(src.empty())
        {
            cout << "open picture error!" << endl;
            return -1;
        }
        
        //分离
        Mat dst,src_hsv;
        vector<Vec3f> circles;
        cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);
        inRange(src_hsv, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255), dst);

        Canny(dst, dst,123,225,3);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;
        cv::findContours(dst, contours, hierachy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        

        for(int i=0;i<contours.size();i++)
        {
            int maxX = 0,maxY = 0,minX = 1000,minY = 1000,Isround = 0;
        
            //绘制出contours向量内所有的像素点   
            for(int j=0;j<contours[i].size();j++) 
            {
                Point P=Point(contours[i][j].x,contours[i][j].y);
                if(maxX < contours[i][j].x) maxX = contours[i][j].x;
                if(maxY < contours[i][j].y) maxY = contours[i][j].y;
                if(minX > contours[i][j].x) minX = contours[i][j].x;
                if(minY > contours[i][j].y) minY = contours[i][j].y;
                circle(src,P,1,Scalar(255,0,0),-1);
            }
            int cenX = (maxX+minX)/2;
            int cenY = (maxY+minY)/2;
            
            vector<double> distances;
            
            for(int j=0;j<contours[i].size();j++) 
            {
                double distance = sqrt( (contours[i][j].x - cenX)*(contours[i][j].x - cenX) + (contours[i][j].y - cenY)*(contours[i][j].y - cenY) );
                distances.push_back(distance);
            }

            if(VectorVar(distances) < 10)
            {
                putText(src,"Round",Point(cenX,cenY),1,2,Scalar(255,0,0),1,8);
                circle(src,Point(cenX,cenY),1,Scalar(255,0,0),-1);
            }
            
    
        }


        imshow("video-demo",src);
        if (waitKey(16) >= 0) break;
        
    }
    video.release();
    return 0;
}