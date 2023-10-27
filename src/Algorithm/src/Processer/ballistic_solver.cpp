//
// Created by nuc12 on 23-7-18.
//

#include "Processer/ballistic_solver.hpp"


namespace processer
{
    BallisticSolver::BallisticSolver() {
        cv::FileStorage fs("./src/Algorithm/configure/Processer/ballistic_solver/params.xml", cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cout << "open processer ballistic_solver param fail" << std::endl;
            exit(0);
        }

        fs["level_one_first"] >> ballistic_param_.level_one_first;
        fs["level_two_first"] >> ballistic_param_.level_two_first;
        fs["level_two_second"] >> ballistic_param_.level_two_second;
        fs["level_three_first"] >> ballistic_param_.level_three_first;
        fs["level_three_second"] >> ballistic_param_.level_three_second;
        fs["level_three_third"] >> ballistic_param_.level_three_third;
        fs["level_three_fourth"] >> ballistic_param_.level_three_fourth;

        fs["k"] >> ballistic_param_.k;
        fs["g"] >> ballistic_param_.g;
        fs["bullet_speed"] >> bullet_speed_;

        fs.release();


    }

    cv::Point3f BallisticSolver::getAngleTime(cv::Point3f position)
    {

        this->setBS_coeff(position);
        double dy, angle, y_actual;
        double t_actual = 0.0;
        double y_temp = position.z / 1000.0;
        double y = y_temp;
        double x = sqrt(position.x * position.x + position.y * position.y) / 1000.0;

        for (int i = 0; i < 40; i++) {
            angle = atan2(y_temp, x);
            t_actual = (exp(this->ballistic_param_.k * x) - 1.0) /
                       (this->ballistic_param_.k * this->bs_coeff * this->bullet_speed_ * cos(angle));
            y_actual = double(bs_coeff * bullet_speed_ * sin(angle) * t_actual - this->ballistic_param_.g * t_actual * t_actual / 2.0);
            dy = y - y_actual;
            y_temp += dy;
            if (abs(dy) < 0.001)
                break;
        }

        float pitch = (angle) / M_PI * 180.0;
        float yaw = atan2(position.y,position.x)/CV_PI*180.0;

        return cv::Point3f(pitch,yaw,t_actual);

    }



    bool BallisticSolver::setBulletSpeed(int bullet_speed)
    {
        this->bullet_speed_ = bullet_speed;
        return true;

    }

    void BallisticSolver::setBS_coeff(cv::Point3f position)
    {
        if (bullet_speed_ < 16)
        {
            bs_coeff = ballistic_param_.level_one_first;
        }
        else if (bullet_speed_ > 17 && bullet_speed_ < 20)
        {
            bs_coeff = ballistic_param_.level_two_first;
            if (position.z > 250)
                bs_coeff *= ballistic_param_.level_two_second;
        }
        else if (bullet_speed_ > 28 && bullet_speed_ < 32)
        {
                bs_coeff = ballistic_param_.level_three_first;
                if (position.z >= 400 && position.z < 900)
                    bs_coeff = ballistic_param_.level_three_second;
                else if(position.z >= 900 && position.z < 1200)
                    bs_coeff = ballistic_param_.level_three_third;
                else if (position.z >= 1200)
                    bs_coeff = ballistic_param_.level_three_fourth;
        }
        else
            bs_coeff = 0.9;
        
        std::cout<<"bs_coeff:"<<bs_coeff<<std::endl;

    }

}