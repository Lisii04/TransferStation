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

static vector<Point2f> high_image_points;
static vector<Point2f> high_model_points;

static vector<Point2f> low_image_points;
static vector<Point2f> low_model_points;

int image_count = 0;
int model_count = 0;

class Calibrator
{
public:
    Mat_<float> high_transform_martix;
    Mat_<float> low_transform_martix;

private:
    // 透视变换
    Mat GetPerspectiveTransform(vector<Point2f> image_points, vector<Point2f> model_points)
    {
        return getPerspectiveTransform(image_points, model_points);
    };

    static void SetCameraPoint(int event, int x, int y, int flags, void *param)
    {
        if (image_count == 8)
            return;
        if (event == EVENT_LBUTTONDOWN)
        {
            switch (image_count / 4)
            {
            case 0:
                cout << "Camera Point High" << image_count + 1 << ": " << Point(x, y) << endl;
                high_image_points.push_back(Point2f(x, y));
                putText(camera_image, to_string(image_count + 1), Point(x, y), 1, 5, Scalar(0, 255, 0), 2);
                circle(camera_image, Point(x, y), 5, Scalar(0, 255, 0), -1);
                imshow("camera", camera_image);
                image_count++;
                break;
            case 1:
                cout << "Camera Point Low" << image_count - 4 + 1 << ": " << Point(x, y) << endl;
                low_image_points.push_back(Point2f(x, y));
                putText(camera_image, to_string(image_count - 4 + 1), Point(x, y), 1, 5, Scalar(0, 0, 255), 2);
                circle(camera_image, Point(x, y), 5, Scalar(0, 0, 255), -1);
                imshow("camera", camera_image);
                image_count++;
            default:
                break;
            }
        }
    };

    static void SetMinimapPoint(int event, int x, int y, int flags, void *param)
    {
        if (model_count == 8)
            return;
        if (event == EVENT_LBUTTONDOWN)
        {
            switch (model_count / 4)
            {
            case 0:
                cout << "Map Point High" << model_count + 1 << ": " << Point(x, y) << endl;
                high_model_points.push_back(Point2f(x, y));
                putText(minimap_image, to_string(model_count + 1), Point(x, y), 1, 5, Scalar(0, 255, 0), 2);
                circle(minimap_image, Point(x, y), 5, Scalar(0, 255, 0), -1);
                imshow("minimap", minimap_image);
                model_count++;
                break;
            case 1:
                cout << "Map Point Low" << model_count - 4 + 1 << ": " << Point(x, y) << endl;
                low_model_points.push_back(Point2f(x, y));
                putText(minimap_image, to_string(model_count - 4 + 1), Point(x, y), 1, 5, Scalar(0, 0, 255), 2);
                circle(minimap_image, Point(x, y), 5, Scalar(0, 0, 255), -1);
                imshow("minimap", minimap_image);
                model_count++;
            default:
                break;
            }
        }
    };

public:
    Calibrator(){}; // 构造函数

    // 透视变换标定
    void MapCalibration(Mat camera_image, Mat minimap_image)
    {
        namedWindow("camera", 0);
        namedWindow("minimap", 0);
        resizeWindow("camera", Size(800, 600));
        resizeWindow("minimap", Size(800, 600));

        setMouseCallback("camera", SetCameraPoint);
        setMouseCallback("minimap", SetMinimapPoint);

        while (true)
        {
            imshow("camera", camera_image);
            imshow("minimap", minimap_image);
            if (waitKey())
            {
                // image_count = 0;
                // model_count = 0;
                break;
            }
        }
        this->high_transform_martix = GetPerspectiveTransform(high_image_points, high_model_points);
        this->low_transform_martix = GetPerspectiveTransform(low_image_points, low_model_points);
    };
};

int main()
{

    Calibrator calibrator;
    calibrator.MapCalibration(camera_image, minimap_image);

    string filename = "../Datas/martixs.yaml"; // 文件的名称
    FileStorage fwriter(filename, FileStorage::WRITE);

    // 存入矩阵Mat类型的数据
    fwriter.write("high_transform_martix", calibrator.high_transform_martix); // 使用write()函数写入数据
    fwriter.write("low_transform_martix", calibrator.low_transform_martix);   // 使用write()函数写入数据

    fwriter.release();

    destroyAllWindows();
}