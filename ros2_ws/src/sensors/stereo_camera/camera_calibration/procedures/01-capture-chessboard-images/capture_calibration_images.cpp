#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

static bool ensure_dir(const char* path)
{
    struct stat st;
    if (stat(path, &st) == 0 && S_ISDIR(st.st_mode))
        return true;
    return mkdir(path, 0755) == 0;
}

int main() 
{
    if (!ensure_dir("images") || !ensure_dir("images/l") || !ensure_dir("images/r"))
    {
        printf("Failed to create images/l and images/r directories.\n");
        return -1;
    }

    VideoCapture cam0("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
    VideoCapture cam1("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
    if(!cam0.isOpened())
    {
       printf("cam0 is not opened.\n");
        return -1;
    }
    if(!cam1.isOpened())
    {
       printf("cam1 is not opened.\n");
        return -1;
    }


    auto loop_start = std::chrono::steady_clock::now();
    auto last_save = loop_start;
    int save_count = 0;
    int last_countdown = -1;
    bool capture_started = false;

    while(1)
    {
        Mat frame_left, frame_right;
        // cam0 >> frame0;
        // cam1 >> frame1;

        cam0.grab();
        cam1.grab();
        // this allows better syncing by grabbing first, then decoding after
        cam0.retrieve(frame_left);
        cam1.retrieve(frame_right);
        
        imshow("right", frame_right);
        imshow("left", frame_left);

        auto now = std::chrono::steady_clock::now();
        auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(now - loop_start).count();

        if (elapsed_sec < 5)
        {
            int countdown = 5 - static_cast<int>(elapsed_sec);
            if (countdown != last_countdown)
            {
                printf("Starting in %d...\n", countdown);
                fflush(stdout);
                last_countdown = countdown;
            }
        }
        else if (!capture_started)
        {
            printf("Starting capture.\n");
            fflush(stdout);
            capture_started = true;
            last_save = now;
        }
        else if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save).count() >= 5000)
        {
            imwrite("images/r/right" + std::to_string(save_count) + ".jpg", frame_right);
            imwrite("images/l/left" + std::to_string(save_count) + ".jpg", frame_left);
            printf("Saved frame %d\n", save_count);
            fflush(stdout);
            save_count++;
            last_save = now;
        }

        if((char)waitKey(1) == 27)
        {
            printf("Stopped.\n");
            break;
        }
    }

    return 0;
}
