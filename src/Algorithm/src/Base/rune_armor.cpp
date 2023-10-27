#include "Base/rune_armor.hpp"
#include <opencv2/imgproc.hpp>

namespace base
{
    RuneArmor::RuneArmor(std::vector<cv::Point2f> hull)
    {
        initRuneArmor(hull);
        findDirection();
    }

    RuneArmor::RuneArmor(vector<Point2d> dl_points_)
    {
        this->have_correct_points = false;
        if(dl_points_.size() > 4)
        {
            for(int i = 0; i < dl_points_.size(); i++)
            {
                if(i == 0)
                    this->dl_points[2] = dl_points_[i];
                else if(i == 1)
                    this->dl_points[1] = dl_points_[i];
                else if(i == 2)
                    this->dl_points[0] = dl_points_[i];
                else if(i == 4)
                    this->dl_points[3] = dl_points_[i];
            }
            this->dl_armor_center = (dl_points[0] + dl_points[1] + dl_points[2] + dl_points[3]) / 4;
            this->have_correct_points = true;
        }
    }

    void RuneArmor::initRuneArmor(std::vector<cv::Point2f> hull)
    {
        this->candidates_points = hull;
        this->vane_rrect = cv::minAreaRect(this->candidates_points);
        cv::Point2f temp_points[4];
        vane_rrect.points(temp_points);//从RotatedRect类中提取出角点
        //对这些点进行矫正;0-1/2-3为长边,1-2/3-0为短边
        this->vane_long_side = calDistance( temp_points[0], temp_points[1]);
        this->vane_short_side = calDistance( temp_points[1], temp_points[2]);

        if (this->vane_long_side > this->vane_short_side)
        {
            for (int i = 0; i < 4; i++)
            {
                this->correct_points[i] = temp_points[i];
            }
        }
        else
        {
            std::swap(this->vane_long_side, this->vane_short_side);
            for (int i = 0; i < 4; i++)
            {
                this->correct_points[i] = temp_points[(i + 1) % 4];
            }
        }        
    }

    void RuneArmor::findDirection()
    {
        Point2f short_side_point1 = (correct_points[1] + correct_points[2]) / 2;
        Point2f short_side_point2 = (correct_points[0] + correct_points[3]) / 2;

        // 计算质心
        Moments mu;
        mu = moments(candidates_points, false);
        Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    
        float temp_side_distance1 = calDistance(mc, short_side_point1);        
        float temp_side_distance2 = calDistance(mc, short_side_point2);     
    
        if(temp_side_distance1 > temp_side_distance2)
            this->find_circle_center_direction = short_side_point1 - short_side_point2;
        else
            this->find_circle_center_direction = short_side_point2 - short_side_point1;            
    }

    float RuneArmor::calDistance(Point2f pt1, Point2f pt2)
    {
        cv::Point2f dis = pt1 - pt2;
        return sqrt(pow(dis.x,2)+pow(dis.y,2));
    }


    void RuneArmor::getPoints(std::vector<cv::Point2f>& pts)
    {
        for(int i = 0; i < 4; i++)
            pts.push_back(points[i]);
    }

}