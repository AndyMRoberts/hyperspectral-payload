#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

struct TimingStats
{
    double min_ms = 1e9;
    double max_ms = -1e9;
    double sum_ms = 0.0;
    double sum_sq_ms = 0.0;
    int count = 0;

    void add(double value_ms)
    {
        min_ms = std::min(min_ms, value_ms);
        max_ms = std::max(max_ms, value_ms);
        sum_ms += value_ms;
        sum_sq_ms += value_ms * value_ms;
        count++;
    }

    double mean_ms() const
    {
        return count > 0 ? sum_ms / count : 0.0;
    }

    double stddev_ms() const
    {
        if (count < 2)
            return 0.0;
        const double mean = mean_ms();
        const double variance = (sum_sq_ms / count) - (mean * mean);
        return std::sqrt(std::max(0.0, variance));
    }
};

static double to_ms(const std::chrono::steady_clock::duration & duration)
{
    return std::chrono::duration<double, std::milli>(duration).count();
}

int main()
{
    VideoCapture cam0(
        "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, "
        "format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=2 ! "
        "video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink",
        cv::CAP_GSTREAMER);
    VideoCapture cam1(
        "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, "
        "format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=2 ! "
        "video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink",
        cv::CAP_GSTREAMER);

    if (!cam0.isOpened())
    {
        printf("cam0 is not opened.\n");
        return -1;
    }
    if (!cam1.isOpened())
    {
        printf("cam1 is not opened.\n");
        return -1;
    }

    printf("Checking stereo camera sync (grab/retrieve timing).\n");
    printf("Press ESC to stop and print summary.\n\n");

    TimingStats grab_skew_stats;
    TimingStats retrieve_skew_stats;
    TimingStats pair_skew_stats;
    TimingStats loop_period_stats;

    auto loop_start = std::chrono::steady_clock::now();
    auto last_loop_end = loop_start;
    int frame_index = 0;
    const int print_every = 20;

    while (1)
    {
        Mat frame_left, frame_right;

        const auto grab0_start = std::chrono::steady_clock::now();
        cam0.grab();
        const auto grab0_end = std::chrono::steady_clock::now();

        const auto grab1_start = grab0_end;
        cam1.grab();
        const auto grab1_end = std::chrono::steady_clock::now();

        const auto retrieve0_start = grab1_end;
        cam0.retrieve(frame_left);
        const auto retrieve0_end = std::chrono::steady_clock::now();

        const auto retrieve1_start = retrieve0_end;
        cam1.retrieve(frame_right);
        const auto retrieve1_end = std::chrono::steady_clock::now();

        const auto loop_end = retrieve1_end;
        const double grab_skew_ms = to_ms(grab1_end - grab0_end);
        const double retrieve_skew_ms = to_ms(retrieve1_end - retrieve0_end);
        const double pair_skew_ms = to_ms(retrieve1_end - grab0_start);
        const double loop_period_ms = to_ms(loop_end - last_loop_end);

        grab_skew_stats.add(grab_skew_ms);
        retrieve_skew_stats.add(retrieve_skew_ms);
        pair_skew_stats.add(pair_skew_ms);
        if (frame_index > 0)
            loop_period_stats.add(loop_period_ms);

        if (frame_index % print_every == 0)
        {
            printf(
                "frame %4d | grab skew: %7.2f ms | retrieve skew: %7.2f ms | "
                "pair latency: %7.2f ms | loop period: %7.2f ms\n",
                frame_index,
                grab_skew_ms,
                retrieve_skew_ms,
                pair_skew_ms,
                frame_index > 0 ? loop_period_ms : 0.0);
            fflush(stdout);
        }

        imshow("left", frame_left);
        imshow("right", frame_right);

        last_loop_end = loop_end;
        frame_index++;

        if ((char)waitKey(1) == 27)
        {
            printf("\nStopped.\n");
            break;
        }
    }

    if (frame_index == 0)
        return 0;

    printf("\nSummary over %d frames:\n", frame_index);
    printf(
        "  grab skew (cam1 grab after cam0):     min=%7.2f  mean=%7.2f  std=%7.2f  max=%7.2f ms\n",
        grab_skew_stats.min_ms,
        grab_skew_stats.mean_ms(),
        grab_skew_stats.stddev_ms(),
        grab_skew_stats.max_ms);
    printf(
        "  retrieve skew (cam1 after cam0):        min=%7.2f  mean=%7.2f  std=%7.2f  max=%7.2f ms\n",
        retrieve_skew_stats.min_ms,
        retrieve_skew_stats.mean_ms(),
        retrieve_skew_stats.stddev_ms(),
        retrieve_skew_stats.max_ms);
    printf(
        "  pair latency (cam0 grab -> cam1 done):  min=%7.2f  mean=%7.2f  std=%7.2f  max=%7.2f ms\n",
        pair_skew_stats.min_ms,
        pair_skew_stats.mean_ms(),
        pair_skew_stats.stddev_ms(),
        pair_skew_stats.max_ms);
    if (loop_period_stats.count > 0)
    {
        printf(
            "  loop period (frame-to-frame):           min=%7.2f  mean=%7.2f  std=%7.2f  max=%7.2f ms\n",
            loop_period_stats.min_ms,
            loop_period_stats.mean_ms(),
            loop_period_stats.stddev_ms(),
            loop_period_stats.max_ms);
    }

    return 0;
}
