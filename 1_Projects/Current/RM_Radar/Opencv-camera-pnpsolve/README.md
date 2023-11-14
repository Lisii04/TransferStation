# 基于单目相机的坐标解算

## 任务：
根据相机画面和实际坐标对应的已知的四个坐标点计算出变换矩阵，从而实现给定相机画面任意坐标计算出实际坐标


### 相机图像坐标系和世界坐标系的关系

```math
s \ast
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
=M \times ( R \times 
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
+ T )
```
其中s是未知量,$\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]$是相机图像坐标系的一点,M是相机的内参矩阵,R是旋转矩阵,$\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ]$是世界坐标系对应的坐标,T是平移矩阵

所以，我们只需要先求出相机的内参矩阵，然后根据几组点利用pnp算法解算出对应的坐标变换(旋转矩阵+平移矩阵)，就可以获取相机画面上一点将其转换为世界坐标系下的三维坐标

### PART1 相机标定

标定相机的内参矩阵 ```camera_matrix``` 和畸变系数 ```dist_coeffs```

- 手动截取给定视频/摄像头画面中的棋盘 (截取的画面按顺序命名，存放在程序目录下)
```
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture inputVideo(YOUR_RESOURCE);
    if (!inputVideo.isOpened())
    {
        cout << "Could not open the input video " << endl;
        return -1;
    }
    inputVideo.set(cv::CAP_PROP_FPS, 60);
    namedWindow("Camera", 0);
    resizeWindow("Camera", cv::Size(800, 600));

    Mat frame;
    string imgname;
    int f = 1;
    while (1)
    {
        inputVideo >> frame;
        if (frame.empty())
            break;
        imshow("Camera", frame);
        char key = waitKey(0);
        if (key == 'q' || key == 'Q') // 退出运行
            break;
        if (key == 'k' || key == 'K') // 截取图片
        {
            cout << "frame:" << f << endl;
            imgname = to_string(f++) + ".jpg";
            imwrite(imgname, frame);
        }
    }
    cout << "Finished writing" << endl;
    return 0;
}

```
- 根据给定图片棋盘格标定相机
```
#include <opencv4/opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

// 定义棋盘格的尺寸
int CHECKERBOARD[2]{YOUR_SIZE_ROWS, YOUR_SIZE_COLS};

int main()
{
    cv::namedWindow("Image", 0);
    cv::resizeWindow("Image", cv::Size(800, 600));

    // 创建矢量以存储每个棋盘图像的三维点矢量
    std::vector<std::vector<cv::Point3f>> objpoints;

    // 创建矢量以存储每个棋盘图像的二维点矢量
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // 为三维点定义世界坐标系
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < CHECKERBOARD[1]; i++)
    {
        for (int j = 0; j < CHECKERBOARD[0]; j++)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    // 提取存储在给定目录中的单个图像的路径
    std::vector<cv::String> images;

    // 包含棋盘图像的文件夹的路径
    std::string path = YOUR_PATH (Example : "../images/*.jpg");

    // 使用glob函数读取所有图像的路径
    cv::glob(path, images);

    cv::Mat frame, gray;

    // 存储检测到的棋盘转角像素坐标的矢量
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // 循环读取图像
    for (int i = 0; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        if (frame.empty())
        {
            continue;
        }
        if (i == 40)
        {
            int b = 1;
        }
        std::cout << "the current image is " << i << "th" << std::endl;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 寻找角点
        // 如果在图像中找到所需数量的角，则success = true
        // opencv4以下版本，flag参数为CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        // 如果检测到所需数量的角点，我们将细化像素坐标并将其显示在棋盘图像上
        if (success)
        {
            // 如果是OpenCV4以下版本，第一个参数为CV_TERMCRIT_EPS | CV_TERMCRIT_ITER
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::Type::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            // 为给定的二维点细化像素坐标
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            // 在棋盘上显示检测到的角点
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

        cv::imshow("Image", frame);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;

    // 通过传递已知3D点（objpoints）的值和检测到的角点（imgpoints）的相应像素坐标来执行相机校准
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    // 内参矩阵
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl
              << std::endl;
    // 透镜畸变系数
    std::cout << "distCoeffs : " << distCoeffs << std::endl
              << std::endl;
    // rvecs
    std::cout << "Rotation vector : " << R << std::endl
              << std::endl;
    // tvecs
    std::cout << "Translation vector : " << T << std::endl
              << std::endl;

    return 0;
}
```
相机标定完成，获得相机的内参矩阵```camera_matrix``` 和畸变系数 ```dist_coeffs```

### PART2 PNP解算
根据相机画面和实际坐标对应的已知的四个坐标点计算出变换矩阵

```
// 相机内参矩阵和畸变系数均由相机标定结果得出
// 相机内参矩阵
Mat camera_matrix = YOUR_CAMERA_MATRIX;
// 相机畸变系数
Mat dist_coeffs = YOUR_DIST_COEFFS;

// 定义旋转向量
Mat rotation_vector;
// 定义平移向量
Mat translation_vector;

// 2D 特征点像素坐标，可以用鼠标事件画出特征点
vector<Point2d> image_points;
// 3D 特征点世界坐标，与像素坐标对应，单位是mm
vector<Point3d> model_points;

// pnp求解
solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
        rotation_vector, translation_vector, 0, SOLVEPNP_ITERATIVE);
// 可选方法有ITERATIVE，EPNP,P3P

```
得到参数：旋转向量```rotation_vector ```、平移向量``` translation_vector ```

```
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
```
得到参数：旋转矩阵```rotMat```、平移矩阵``` Tvec```
- 接下来计算 $s$

因为

```math
s \ast
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
=M \times ( R \times 
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
+ T )
```
其实就是
```math
\left [ \begin{matrix}
    s \ast u  \\
    s \ast v  \\
    s
   \end{matrix}\right ]
=M \times ( R \times 
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
+ T )
```
所以只要根据已知 $M$ 和 $T$ 以及一个世界坐标系下的点 $\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] $ ，求出相机图像坐标系的一点 $\left [ \begin{matrix}
    s \ast u  \\
    s \ast v  \\
    s
   \end{matrix}\right ]$ 就可以求出 $s$

```
Mat W = (Mat_<double>(3, 1) << model_points[0].x, model_points[0].y, model_points[0].z);
// W是世界坐标系对应的坐标，这里采用第一组用于pnp解算的点
W.convertTo(W, CV_32F);

Mat U = (camera_matrix * (rotMat * W + Tvec));
// U是相机图像坐标系待求点
double s = U.at<float>(0, 2);
```
这时所有未知量都已经求出，可以进行重映射(获取相机画面上一点将其转换为世界坐标系下的三维坐标)

### PART3 重映射

因为
```math
s \ast
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
=M \times ( R \times 
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
+ T )
```
所以

```math
M^{-1} \ast s \times 
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
= R \ast
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
+ T
```

```math
M^{-1} \ast s \times 
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
- T= R \ast
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
```

```math
R^{-1} \ast ( M^{-1} \ast s \times 
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
- T) = 
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
```
即
```math
\left [ \begin{matrix}
    x  \\
    y  \\
    z
   \end{matrix}\right ] 
=R^{-1} \ast ( M^{-1} \ast s \times 
\left [ \begin{matrix}
    u  \\
    v  \\
    1
   \end{matrix}\right ]
- T) 
```
所以可以根据任意 ```image_point``` 解得 ```world_point```
```
world_point = rotMat.inv() * (s * camera_matrix.inv() * image_point - Tvec);
```

### 搞定！
