#ifndef COMMON_HPP_
#define COMMON_HPP_

// #include <iostream>
#include <vector>
#include "opencv4/opencv2/opencv.hpp"

// class DetectResult
// {
// public:
//     cv::Rect2d bbox_;
//     cv::Point2d f_point_[4];
//     int id_;
//     double conf_;
//     DetectResult(cv::Rect2d bbox, cv::Point2d* f_point, int id, double conf) : bbox_(bbox), id_(id), conf_(conf)
//     {
//         memcpy(f_point_,f_point,4*sizeof(cv::Point2d));
//     }
// };
struct DetectResult
{
    cv::Rect2d bbox;
    std::vector<cv::Point2d> points;
    int id;
    double conf;
};

using DetectResultList = std::vector<DetectResult>;
using SegmentResultList = std::vector<cv::Mat>;


#endif