#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

class Calibrator
{
public:
    Mat_<float> high_transform_martix;
    Mat_<float> low_transform_martix;

private:
    // 透视变换
    Mat GetPerspectiveTransform(vector<Point2f> image_points, vector<Point2f> model_points);

    static void SetCameraPoint(int event, int x, int y, int flags, void *param);

    static void SetMinimapPoint(int event, int x, int y, int flags, void *param);

public:
    Calibrator(){}; // 构造函数

    // 透视变换标定
    void MapCalibration(Mat camera_image, Mat minimap_image);
};
