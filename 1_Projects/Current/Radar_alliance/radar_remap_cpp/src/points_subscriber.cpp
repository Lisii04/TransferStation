#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

string minimap_image_path = "./Images/minimap.png";
Mat minimap_image = imread(minimap_image_path);
string roi_image_path = "./Images/roi.jpg";
Mat roi_image = imread(roi_image_path);
Mat temp_image = imread(minimap_image_path);

Mat_<float> high_transform_martix;
Mat_<float> low_transform_martix;

Mat_<float> image_point(3, 1);
Mat_<float> world_point_high(3, 1);
Mat_<float> world_point_low(3, 1);

std::vector<cv::Point2f> points;

long frame_count = 0;

void remap(vector<Point2f> points)
{

    minimap_image.copyTo(temp_image);
    try {
        for (size_t i = 0; i < points.size(); i++) {
            // ******* 由像素坐标计算到世界坐标 *********
            image_point = (Mat_<float>(3, 1) << points[i].x, points[i].y, 1);

            world_point_high = high_transform_martix * image_point;
            world_point_low = low_transform_martix * image_point;

            Point _world_point_high = Point(world_point_high.at<float>(0, 0) / world_point_high.at<float>(0, 2), world_point_high.at<float>(1, 0) / world_point_high.at<float>(0, 2));
            Point _world_point_low = Point(world_point_low.at<float>(0, 0) / world_point_low.at<float>(0, 2), world_point_low.at<float>(1, 0) / world_point_low.at<float>(0, 2));
            // ******* 计算结束 *********

            // 筛选负值
            if (_world_point_high.x > 0 && _world_point_high.y > 0 && _world_point_low.x > 0 && _world_point_low.y > 0) {
                // ******* 判断是否在高地 并绘制坐标 ********
                if ((int)(roi_image.at<Vec3b>(_world_point_high.y, _world_point_high.x)[0]) > 150) {
                    circle(temp_image, Point(_world_point_high.x, _world_point_high.y), 20, Scalar(0, 0, 255), -1);
                } else {
                    circle(temp_image, Point(_world_point_low.x, _world_point_low.y), 20, Scalar(0, 0, 255), -1);
                }
            }
            // ******* 结束 ********
        }
        imshow("1", temp_image);
        waitKey(1);
        frame_count++;
    } catch (const std::exception& e) {
        cout << "\033[31m[ERROR]\033[0m";
        std::cerr << e.what() << '\n';
    }
}

class Points_subscriber : public rclcpp::Node {
public:
    Points_subscriber(std::string name)
        : Node(name)
    {
        cout << ">\033[32m[DONE]\033[0m[节点启动]\n>\033[34m[WAITING]\033[0m\033[5m[等待数据]\033[0m" << endl;
        // 创建一个订阅者订阅话题
        command_subscribe_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("points_data", 10, std::bind(&Points_subscriber::command_callback, this, std::placeholders::_1));
    }

private:
    // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_subscribe_;
    // 收到话题数据的回调函数
    void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        points.clear();
        cout << "\033c>\033[33m[WORKING]\033[0m[正在接收点坐标]\033[?25l" << endl;
        if (msg->data.size() == 0) {
            cout << "\033[31m[ERROR]\033[0m[未识别到坐标]" << endl;
        }

        for (auto i = 0; i < msg->data.size() - 1; i += 2) {
            printf("(%0.1f,%0.1f)", msg->data.data()[i], msg->data.data()[i + 1]);
            points.push_back(cv::Point2f(msg->data.data()[i], msg->data.data()[i + 1]));
        }
        cout << "\n"
             << endl;
        remap(points);
    }
};

int main(int argc, char** argv)
{
    cout << "\033[1m[正在启动小地图映射节点]\033[0m" << endl;
    cout << ">\033[33m[WORKING]\033[0m[初始化|读取地图和高地数据]" << endl;
    cout << ">\033[32m[DONE]\033[0m[读取结束]" << endl;
    // ******* 读取相机标定矩阵 *******
    string filename = "./Datas/martixs.yaml";

    cout << ">\033[33m[WORKING]\033[0m[读取相机标定矩阵]" << endl;

    // 以读取的模式打开相机标定文件
    FileStorage fread(filename, FileStorage::READ);
    // 判断是否打开成功
    if (!fread.isOpened()) {
        cout << ">\033[31m[ERROR]\033[0m[打开文件失败，请确认文件名称是否正确]" << endl;
        return -1;
    }

    // 读取Mat类型数据
    fread["high_transform_martix"] >> high_transform_martix;
    fread["low_transform_martix"] >> low_transform_martix;

    // 关闭文件
    fread.release();

    cout << ">\033[32m[DONE]\033[0m[读取结束]" << endl;
    // ****** 读取结束 ******

    // 调整ROI图像大小
    resize(roi_image, roi_image, minimap_image.size());

    namedWindow("1", 0);
    resizeWindow("1", Size(800, 500));

    cout << ">\033[33m[WORKING]\033[0m[启动节点]" << endl;
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Points_subscriber>("points_subscriber");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
