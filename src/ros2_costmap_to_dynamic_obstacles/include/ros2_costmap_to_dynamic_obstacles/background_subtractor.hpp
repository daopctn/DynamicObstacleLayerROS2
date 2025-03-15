#ifndef BACKGROUND_SUBTRACTOR_HPP
#define BACKGROUND_SUBTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BackgroundSubtractor
{
public:
    BackgroundSubtractor();
    void apply(const cv::Mat &costmap, cv::Mat &fg_mask);

private:
    cv::Mat slow_background;
    cv::Mat fast_background;
    float alpha_fast = 0.1;
    float alpha_slow = 0.01;
    float threshold1 = 50;
    float threshold2 = 20;
};

#endif // BACKGROUND_SUBTRACTOR_HPP
