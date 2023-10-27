//
// Created by Wang on 23-6-18.
//

#ifndef RMOS_PNP_SOLVER_HPP
#define RMOS_PNP_SOLVER_HPP

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include "../../../Base/armor.hpp"
#include "../../../Base/rune_armor.hpp"
#include "../../detector_interfaces/solver_interface.hpp"


namespace detector
{
    class PnpSolver : public SolverInterface
    {
    public:
        PnpSolver()
        {
            small_armor.push_back(cv::Point3f(0.0,small_width / 2.0, -small_height / 2.0));
            small_armor.push_back(cv::Point3f(0.0,small_width / 2.0, small_height / 2.0));
            small_armor.push_back(cv::Point3f(0.0,-small_width / 2.0, small_height / 2.0));
            small_armor.push_back(cv::Point3f(0.0,-small_width / 2.0, -small_height / 2.0));

            big_armor.push_back(cv::Point3d(0.0,big_width / 2.0, -big_height / 2.0));
            big_armor.push_back(cv::Point3d(0.0,big_width / 2.0,  big_height / 2.0));
            big_armor.push_back(cv::Point3d(0.0,-big_width / 2.0, big_height / 2.0));
            big_armor.push_back(cv::Point3d(0.0,-big_width / 2.0, -big_height / 2.0));

            rune_armor.push_back(cv::Point3d(0.0,rune_width / 2.0, -rune_height / 2.0));
            rune_armor.push_back(cv::Point3d(0.0,rune_width / 2.0, rune_height / 2.0));
            rune_armor.push_back(cv::Point3d(0.0,-rune_width / 2.0, rune_height / 2.0));
            rune_armor.push_back(cv::Point3d(0.0,-rune_width / 2.0, -rune_height / 2.0));
        };
        ~PnpSolver(){};

        bool solveArmorPose(const base::Armor& armor,const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs,cv::Mat &tVec, cv::Mat &rVec) override;

        bool solveRuneArmorPose(std::vector<cv::Point2f> rune_next_pos,const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs,cv::Mat &tVec, cv::Mat &rVec);

    private:
        float small_width = 125;
        float small_height = 55;
        float big_width = 225;
        float big_height = 55;
        float rune_width = 253.42;
        float rune_height = 270.49;
        std::vector<cv::Point3f> small_armor;
        std::vector<cv::Point3f> big_armor;
        std::vector<cv::Point3f> rune_armor;

    };


}
#endif //RMOS_PNP_SOLVER_HPP
