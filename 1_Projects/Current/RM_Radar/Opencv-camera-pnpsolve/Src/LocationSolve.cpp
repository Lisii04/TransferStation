#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

string image_path_1 = "../Images/camera2.png";
Mat camera_image = imread(image_path_1);
string image_path_2 = "../Images/minimap.png";
Mat minimap_image = imread(image_path_2);

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
            // Mat temp_1 = imread(image_path_2);
            // Mat temp_2 = imread(image_path_1);

            image_point = (Mat_<float>(3, 1) << x, y, 1);

            world_point_high = high_transform_martix * image_point;
            world_point_low = low_transform_martix * image_point;

            Point _world_point_high = Point(world_point_high.at<float>(0, 0) / world_point_high.at<float>(0, 2), world_point_high.at<float>(1, 0) / world_point_high.at<float>(0, 2));
            Point _world_point_low = Point(world_point_low.at<float>(0, 0) / world_point_low.at<float>(0, 2), world_point_low.at<float>(1, 0) / world_point_low.at<float>(0, 2));

            // putText(minimap_image, "HIGH", Point(_world_point_high.x, _world_point_high.y - 60), 1, 5, Scalar(0, 0, 255), 3);
            circle(minimap_image, Point(_world_point_high.x, _world_point_high.y), 10, Scalar(0, 0, 255), -1);
            // circle(minimap_image, Point(_world_point_high.x, _world_point_high.y), 15, Scalar(0, 0, 255), 2);
            // putText(minimap_image, "LOW", Point(_world_point_low.x, _world_point_low.y - 60), 1, 5, Scalar(0, 255, 0), 3);
            circle(minimap_image, Point(_world_point_low.x, _world_point_low.y), 10, Scalar(0, 255, 0), -1);
            // circle(minimap_image, Point(_world_point_low.x, _world_point_low.y), 15, Scalar(0, 255, 0), 2);

            circle(camera_image, Point(x, y), 10, Scalar(0, 0, 255), -1);
            // circle(camera_image, Point(x, y), 15, Scalar(0, 0, 255), 2);

            imshow("3", camera_image);
            imshow("4", minimap_image);
        }
    }

public:
    RobotLocationSolve(){}; // 构造函数

    void remap()
    {
        namedWindow("3", 0);
        resizeWindow("3", Size(800, 600));
        namedWindow("4", 0);
        resizeWindow("4", Size(800, 600));

        setMouseCallback("3", remap, 0);

        imshow("3", camera_image);
        imshow("4", minimap_image);
        waitKey();
    }
};

int main(int argc, char **argv)
{
    string filename = "../Datas/martixs.yaml"; // 文件的名称
    // 以读取的模式打开文件
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

    RobotLocationSolve robotLocationSolve;

    robotLocationSolve.remap();

    return 0;
}
