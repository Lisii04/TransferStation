#include <opencv4/opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

int main()
{
     string image_path = "/home/lisii/Documents/Github-Repos/Current/TransferStation/1_Projects/Current/Opencv-camera-pnpsolve/build/images/2.jpg";
     Mat image = imread(image_path);

     // 2D 特征点像素坐标，这里是用PS找出，也可以用鼠标事件画出特征点
     vector<Point2d> image_points;
     image_points.push_back(Point2d(152, 92));
     image_points.push_back(Point2d(426, 94));
     image_points.push_back(Point2d(428, 394));
     image_points.push_back(Point2d(126, 380));

     // 画出四个特征点
     for (int i = 0; i < image_points.size(); i++)
     {
          circle(image, image_points[i], 3, Scalar(0, 0, 255), -1);
     }

     // 3D 特征点世界坐标，与像素坐标对应，单位是mm
     std::vector<Point3d> model_points;
     model_points.push_back(Point3d(-42.5f, -42.5f, 0)); // 左上角(-42.5mm,-42.5mm)
     model_points.push_back(Point3d(+42.5f, -42.5f, 0));
     model_points.push_back(Point3d(+42.5f, +42.5f, 0));
     model_points.push_back(Point3d(-42.5f, +42.5f, 0));
     // 　注意世界坐标和像素坐标要一一对应

     // 相机内参矩阵和畸变系数均由相机标定结果得出
     // 相机内参矩阵
     Mat camera_matrix = (Mat_<double>(3, 3) << 286.6874365608449, 0, 158.121765495713,
                          0, 298.0282198264642, 121.1850185592787,
                          0, 0, 1);
     // 相机畸变系数
     Mat dist_coeffs = (Mat_<double>(5, 1) << 0.0380483210388743, 0.3756681875627417, -0.004677299138763694,
                        0.0001517564577113271, -1.651688321054696);

     cout << "Camera Matrix " << endl
          << camera_matrix << endl
          << endl;
     // 旋转向量
     Mat rotation_vector;
     // 平移向量
     Mat translation_vector;

     // pnp求解
     solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
              rotation_vector, translation_vector, 0, cv::SOLVEPNP_EPNP);
     // ITERATIVE，EPNP,P3P

     cout << "Rotation Vector " << endl
          << rotation_vector << endl
          << endl;
     cout << "Translation Vector" << endl
          << translation_vector << endl
          << endl;

     Mat Rvec;
     Mat_<float> Tvec;
     rotation_vector.convertTo(Rvec, CV_32F);    // 旋转向量转换格式
     translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式

     Mat_<float> rotMat(3, 3);
     Rodrigues(Rvec, rotMat);
     // 旋转向量转成旋转矩阵
     cout << "rotMat" << endl
          << rotMat << endl
          << endl;

     Mat_<float> image_point = (Mat_<float>(3, 1) << 20, 20, 1);
     Mat_<float> world_point;

     world_point = (rotMat.inv()) * (image_point - Tvec);

     cout << "world_point" << endl
          << world_point << endl;
}
