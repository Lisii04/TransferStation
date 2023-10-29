#include "func.hpp"

int main()
{

    cv::VideoCapture video;
    video.open("7.mp4"); // 打开视频文件/摄像头

    // video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    // std::cout << video.get(cv::CAP_PROP_FRAME_WIDTH) << video.get(cv::CAP_PROP_FRAME_HEIGHT) << video.get(cv::CAP_PROP_FPS) << std::endl;
    // video.set(3, 1920);
    // video.set(4, 1080);

    // video.set(cv::CAP_PROP_FPS, 90);
    // std::cout << video.get(3) << video.get(4) << std::endl;

    VideoProcess(video); // 处理视频帧

    video.release(); // 关闭视频文件/摄像头

    return 0;
}
