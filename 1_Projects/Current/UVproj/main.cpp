#include "func.hpp"

int main()
{
    cv::VideoCapture video;
    video.open("6.mp4"); // 打开视频文件/摄像头

    video.set(cv::CAP_PROP_FPS, 90);

    VideoProcess(video); // 处理视频帧

    video.release(); // 关闭视频文件/摄像头

    return 0;
}
