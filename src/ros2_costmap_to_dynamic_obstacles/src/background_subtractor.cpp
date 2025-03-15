#include "background_subtractor.hpp"

BackgroundSubtractor::BackgroundSubtractor() {}

void BackgroundSubtractor::apply(const cv::Mat &costmap, cv::Mat &fg_mask)
{
    if (slow_background.empty())
    {
        slow_background = costmap.clone();
        fast_background = costmap.clone();
        fg_mask = cv::Mat::zeros(costmap.size(), CV_8UC1);
        return;
    }

    // Update background models
    slow_background = (1 - alpha_slow) * slow_background + alpha_slow * costmap;
    fast_background = (1 - alpha_fast) * fast_background + alpha_fast * costmap;

    // Compute foreground mask
    cv::Mat diff;
    cv::absdiff(fast_background, slow_background, diff);
    cv::threshold(diff, fg_mask, threshold2, 255, cv::THRESH_BINARY);
    
    // Further refine by eliminating noise
    cv::threshold(fast_background, fg_mask, threshold1, 255, cv::THRESH_BINARY);
}
