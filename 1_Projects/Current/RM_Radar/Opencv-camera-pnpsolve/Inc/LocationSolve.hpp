#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

class RobotLocationSolve
{
private:
    // 透视变换

    static void remap(int event, int x, int y, int flags, void *param);

public:
    RobotLocationSolve(){}; // 构造函数

    void remap();
};
