#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

string image_path_1 = "../Images/camera2.png";
Mat camera_image = imread(image_path_1);
string image_path_2 = "../Images/minimap.jpg";
Mat minimap_image = imread(image_path_2);
string image_path_3 = "../Images/roi.jpg";
Mat roi_image = imread(image_path_3);


Mat_<float> high_transform_martix;
Mat_<float> low_transform_martix;

Mat_<float> image_point(3, 1);
Mat_<float> world_point_high(3, 1);
Mat_<float> world_point_low(3, 1);

class RobotLocationSolve
{
private:
    // 透视变换

    static void remap(int event, int x, int y, int flags, void *param)
    {
        if (event == EVENT_MOUSEMOVE)
        { 
            // ******* 由像素坐标计算到世界坐标 *********
            image_point = (Mat_<float>(3, 1) << x, y, 1);

            world_point_high = high_transform_martix * image_point;
            world_point_low = low_transform_martix * image_point;

            Point _world_point_high = Point(world_point_high.at<float>(0, 0) / world_point_high.at<float>(0, 2), world_point_high.at<float>(1, 0) / world_point_high.at<float>(0, 2));
            Point _world_point_low = Point(world_point_low.at<float>(0, 0) / world_point_low.at<float>(0, 2), world_point_low.at<float>(1, 0) / world_point_low.at<float>(0, 2));
            
            // 筛选负值
            if(_world_point_high.x < 0 || _world_point_high.y < 0)
            {
                _world_point_high.x = (_world_point_high.x > 0) ? (_world_point_high.x) : (0);
                _world_point_high.y = (_world_point_high.y > 0) ? (_world_point_high.y) : (0);
            }
            if (_world_point_low.x < 0 || _world_point_low.y < 0)
            {
                _world_point_low.x = (_world_point_low.x > 0) ? (_world_point_low.x) : (0);
                _world_point_low.y = (_world_point_low.y > 0) ? (_world_point_low.y) : (0);
            }
            // ******* 计算结束 *********
            
            // ******* 判断是否在高地 并绘制坐标 ********
            if((int)(roi_image.at<Vec3b>(_world_point_high.y, _world_point_high.x)[0]) > 150)
            {
                circle(minimap_image, Point(_world_point_high.x, _world_point_high.y), 10, Scalar(0, 0, 255), -1);
            }else{
                circle(minimap_image, Point(_world_point_low.x, _world_point_low.y), 10, Scalar(0, 0, 255), -1);
            }

            circle(camera_image, Point(x, y), 10, Scalar(0, 0, 255), -1);
            // ******* 结束 ********

            imshow("3", camera_image);
            imshow("4", minimap_image);
        }
    }

public:
    RobotLocationSolve(){}; // 构造函数

    void remap(VideoCapture video)
    {
        namedWindow("3", 0);
        resizeWindow("3", Size(800, 600));
        namedWindow("4", 0);
        resizeWindow("4", Size(800, 600));

        while (true)
        {
            video >> camera_image;
            if(camera_image.empty())
            {
                break;
            }

            setMouseCallback("3", remap, 0);

            imshow("3", camera_image);
            imshow("4", minimap_image);
            waitKey(1);
        }
    }
};

int main(int argc, char **argv)
{

    // ******* 读取相机标定矩阵 *******
    string filename = "../Datas/martixs.yaml";
    // 以读取的模式打开相机标定文件
    FileStorage fread(filename, FileStorage::READ);
    // 判断是否打开成功
    if (!fread.isOpened())
    {
        cout << "打开文件失败，请确认文件名称是否正确！" << endl;
        return -1;
    }

    // 读取Mat类型数据
    fread["high_transform_martix"] >> high_transform_martix;
    fread["low_transform_martix"] >> low_transform_martix;

    // 关闭文件
    fread.release();
    // ****** 读取结束 ******


    // ******** 读取摄像机画面 **********
    VideoCapture video;
    video.open("../Videos/video.mp4");
    // ******** 读取摄像机画面结束 **********


    // 调整ROI图像大小
    resize(roi_image,roi_image,minimap_image.size());


    // ******* 坐标解算 ********
    RobotLocationSolve robotLocationSolve;

    robotLocationSolve.remap(video);
    // ******* 坐标解算结束 ********

    return 0;
}
