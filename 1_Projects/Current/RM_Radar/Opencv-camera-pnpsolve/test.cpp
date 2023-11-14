#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

string image_path_1 = "../camera2.png";
Mat camera_image = imread(image_path_1);
string image_path_2 = "../minimap.jpg";
Mat minimap_image = imread(image_path_2);

static vector<Point2f> high_image_points;
static vector<Point2f> high_model_points;

static vector<Point2f> low_image_points;
static vector<Point2f> low_model_points;

int image_count = 0;
int model_count = 0;

// test ------------------------------------
Mat_<float> test_high_transform_martix;
Mat_<float> test_low_transform_martix;

Mat test_camera_image = imread(image_path_1);
Mat test_minimap_image = imread(image_path_2);

Mat_<float> image_point(3, 1);
Mat_<float> world_point_high(3, 1);
Mat_<float> world_point_low(3, 1);
// test ------------------------------------
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
                cout << "Camera Point High" << image_count << ": " << Point(x, y) << endl;
                high_image_points.push_back(Point2f(x, y));
                putText(camera_image, to_string(image_count), Point(x, y), 1, 5, Scalar(0, 255, 0), 2);
                circle(camera_image, Point(x, y), 5, Scalar(0, 255, 0), -1);
                imshow("camera", camera_image);
                image_count++;
                break;
            case 1:
                cout << "Camera Point Low" << image_count << ": " << Point(x, y) << endl;
                low_image_points.push_back(Point2f(x, y));
                putText(camera_image, to_string(image_count - 4), Point(x, y), 1, 5, Scalar(0, 0, 255), 2);
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
                cout << "Map Point High" << model_count << ": " << Point(x, y) << endl;
                high_model_points.push_back(Point2f(x, y));
                putText(minimap_image, to_string(model_count), Point(x, y), 1, 5, Scalar(0, 255, 0), 2);
                circle(minimap_image, Point(x, y), 5, Scalar(0, 255, 0), -1);
                imshow("minimap", minimap_image);
                model_count++;
                break;
            case 1:
                cout << "Map Point Low" << model_count << ": " << Point(x, y) << endl;
                low_model_points.push_back(Point2f(x, y));
                putText(minimap_image, to_string(model_count - 4), Point(x, y), 1, 5, Scalar(0, 0, 255), 2);
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

class RobotLocationSolve
{
};

void test_cameramap(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_MOUSEMOVE)
    {
        Mat temp_1 = imread(image_path_2);
        Mat temp_2 = imread(image_path_1);

        image_point = (Mat_<float>(3, 1) << x, y, 1);

        world_point_high = test_high_transform_martix * image_point;
        world_point_low = test_low_transform_martix * image_point;

        Point _world_point_high = Point(world_point_high.at<float>(0, 0) / world_point_high.at<float>(0, 2), world_point_high.at<float>(1, 0) / world_point_high.at<float>(0, 2));
        Point _world_point_low = Point(world_point_low.at<float>(0, 0) / world_point_low.at<float>(0, 2), world_point_low.at<float>(1, 0) / world_point_low.at<float>(0, 2));

        putText(temp_1, "HIGH", Point(_world_point_high.x, _world_point_high.y - 60), 1, 5, Scalar(0, 0, 255), 3);
        circle(temp_1, Point(_world_point_high.x, _world_point_high.y), 10, Scalar(0, 0, 255), -1);
        circle(temp_1, Point(_world_point_high.x, _world_point_high.y), 15, Scalar(0, 0, 255), 2);
        putText(temp_1, "LOW", Point(_world_point_low.x, _world_point_low.y - 60), 1, 5, Scalar(0, 255, 0), 3);
        circle(temp_1, Point(_world_point_low.x, _world_point_low.y), 10, Scalar(0, 255, 0), -1);
        circle(temp_1, Point(_world_point_low.x, _world_point_low.y), 15, Scalar(0, 255, 0), 2);

        circle(temp_2, Point(x, y), 10, Scalar(0, 0, 255), -1);
        circle(temp_2, Point(x, y), 15, Scalar(0, 0, 255), 2);

        imshow("3", temp_2);
        imshow("4", temp_1);
    }
}

int main()
{

    Calibrator calibrator;
    calibrator.MapCalibration(camera_image, minimap_image);

    test_high_transform_martix = calibrator.high_transform_martix;
    test_low_transform_martix = calibrator.low_transform_martix;

    destroyAllWindows();

    namedWindow("3", 0);
    resizeWindow("3", Size(800, 600));
    namedWindow("4", 0);
    resizeWindow("4", Size(800, 600));

    setMouseCallback("3", test_cameramap, 0);

    imshow("3", test_camera_image);
    imshow("4", test_minimap_image);
    waitKey();
}