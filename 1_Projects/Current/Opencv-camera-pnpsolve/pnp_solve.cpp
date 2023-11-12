#include <opencv4/opencv2/opencv.hpp>
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;

string image_path_1 = "../camera.png";
Mat image_1 = imread(image_path_1);
string image_path_2 = "../map2.jpg";
Mat image_2 = imread(image_path_2);
// 2D 特征点像素坐标，这里是用PS找出，也可以用鼠标事件画出特征点
vector<Point2d> image_points;
// 3D 特征点世界坐标，与像素坐标对应，单位是mm
vector<Point3d> model_points;

int image_count = 1;
int model_count = 1;

Mat_<float> image_point(3, 1);
Mat_<float> world_point(3, 1);
// Mat world_point = (Mat_<float>(4, 1) << 0, 0, 0, 1);

void on_click_1(int event, int x, int y, int flags, void *param)
{
     if (event == EVENT_LBUTTONDOWN) // 鼠标移动将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
     {
          cout << image_count << ":" << Point(x, y) << endl;
          image_points.push_back(Point(x, y));
          image_count++;
     }
}

void on_click_2(int event, int x, int y, int flags, void *param)
{
     if (event == EVENT_LBUTTONDOWN) // 鼠标移动将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
     {
          cout << model_count << ":" << Point(x, y) << endl;
          model_points.push_back(Point3d(x, y, 0));
          model_count++;
     }
}

void on_click_3(int event, int x, int y, int flags, void *param)
{
     if (event == EVENT_LBUTTONDOWN) // 鼠标移动将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
     {
          image_point = (Mat_<float>(3, 1) << x, y, 1);
          cout << "image_point" << endl
               << image_point << endl;
     }
}

int main()
{
     namedWindow("1", 0);
     namedWindow("2", 0);
     resizeWindow("1", Size(800, 600));
     resizeWindow("2", Size(800, 600));

     setMouseCallback("1", on_click_1, 0);
     setMouseCallback("2", on_click_2, 0);

     while (true)
     {
          imshow("1", image_1);
          imshow("2", image_2);
          if (waitKey())
          {
               break;
          }
     }

     // 相机内参矩阵和畸变系数均由相机标定结果得出
     // 相机内参矩阵
     Mat camera_matrix = (Mat_<double>(3, 3) << 4209.484117068237, 0.0, 1551.7418129503055, 0.0, 4206.961588317127, 1007.2761954833542, 0.0, 0.0, 1.0);
     // 相机畸变系数
     Mat dist_coeffs = (Mat_<double>(5, 1) << -0.49303853681639626, 1.306694963446007, 0.004516985121496771, 8.986629425496268e-05, -3.416795830838925);

     cout << "Camera Matrix " << endl
          << camera_matrix << endl
          << endl;
     // 旋转向量
     Mat rotation_vector;
     // 平移向量
     Mat translation_vector;

     // pnp求解
     solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
              rotation_vector, translation_vector, 0, SOLVEPNP_ITERATIVE);
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

     cout << "Tvec" << endl
          << Tvec << endl
          << endl;

     camera_matrix.convertTo(camera_matrix, CV_32F);

     /** 计算 s
      *
      * s*U = M * (R * W + T)
      * s是未知量
      * U是相机图像坐标系的一点
      * M是camera_matrix,R是rotMat
      * W是世界坐标系对应的坐标
      * T是Tvec
      */
     // Mat W = (Mat_<double>(3, 1) << model_points[0].x, model_points[0].y, model_points[0].z);
     // W.convertTo(W, CV_32F);

     // Mat U = (camera_matrix * (rotMat * W + Tvec));

     // cout << "U" << endl
     //      << U << endl;

     // double s = U.at<float>(2, 0);

     // cout << "s" << endl
     //      << s << endl
     //      << endl;

     //  R^-1 * (M^-1 * s*U - T) = W
     // W =  rotMat.inv() * (s * camera_matrix.inv() * U - Tvec)

     namedWindow("3", 0);
     resizeWindow("3", Size(800, 600));

     setMouseCallback("3", on_click_3, 0);

     while (true)
     {
          imshow("3", image_1);

          if (waitKey() == 27)
          {
               break;
          }

          Mat leftSideMat = rotMat.inv() * camera_matrix.inv() * image_point;
          Mat rightSideMat = rotMat.inv() * Tvec;

          double s = (rightSideMat.at<double>(2, 0) / leftSideMat.at<double>(2, 0));

          cout << "s" << endl
               << s << endl
               << endl;

          world_point = rotMat.inv() * (s * camera_matrix.inv() * image_point - Tvec);
          Point _world_point = Point(world_point.at<float>(0, 0), world_point.at<float>(1, 0));

          cout << "world_point" << endl
               << _world_point << endl;

          circle(image_2, Point(_world_point.x, _world_point.y), 10, Scalar(0, 0, 255), -1);
          imshow("1", image_2);
          waitKey();
     }
}
