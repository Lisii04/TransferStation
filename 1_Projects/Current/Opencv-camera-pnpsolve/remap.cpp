#include <opencv4/opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

Mat rotMat = (Mat_<double>(3, 3) << -0.45257092, 0.88639301, -0.097401381,
              -0.30590531, -0.05172497, 0.95065582,
              0.83761656, 0.46003476, 0.29456154);
Mat camera_matrix = (Mat_<double>(3, 3) << 4209.484117068237, 0, 1551.741812950306,
                     0, 4206.961588317127, 1007.276195483354,
                     0, 0, 1);
Mat Tvec = (Mat_<double>(3, 1) << -317.41684,
            40.651932,
            623.71454);
double s = 1052.68;

string image_path_1 = "../camera.png";
Mat image_1 = imread(image_path_1);
string image_path_2 = "../map2.jpg";
Mat image_2 = imread(image_path_2);

int image_count = 1;
int model_count = 1;

Mat_<float> image_point(3, 1);
Mat_<float> model_point(3, 1);

Point _image_point;
Point _model_point;

void on_click(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN) // 鼠标移动将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
    {
        cout << image_count << ":" << Point(x, y) << endl;
        image_point = (Mat_<double>(3, 1) << x, y, 1);
        image_count++;
    }
}

int main()
{

    namedWindow("1", 0);
    resizeWindow("1", Size(800, 600));

    namedWindow("2", 0);
    resizeWindow("2", Size(800, 600));

    while (true)
    {
        setMouseCallback("1", on_click, 0);

        camera_matrix.convertTo(camera_matrix, CV_32F);
        rotMat.convertTo(rotMat, CV_32F);
        Tvec.convertTo(Tvec, CV_32F);

        imshow("1", image_1);
        waitKey(1);

        model_point = rotMat.inv() * (s * camera_matrix.inv() * image_point - Tvec);
        _model_point = Point(model_point.at<float>(0, 0), model_point.at<float>(1, 0));

        cout << "model_point" << endl
             << _model_point << endl;

        circle(image_2, Point(_model_point.x, _model_point.y), 10, Scalar(0, 0, 255), -1);

        imshow("2", image_2);
        if (waitKey() == 27)
        {
            break;
        }
    }
}
