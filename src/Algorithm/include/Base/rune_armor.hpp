#ifndef RMOS_BASE__RUNE_ARMOR_HPP_
#define RMOS_BASE__RUNE_ARMOR_HPP_

#include<opencv2/opencv.hpp>
#include "const.hpp"

namespace base
{
    using namespace cv;
    using namespace std;
    
    class RuneArmor
    {
    public:
        RuneArmor() = default;
        RuneArmor(std::vector<cv::Point2f> after_approx_points);    // approxPDP返回点集传入
        RuneArmor(vector<Point2d> dl_points_);



        void getPoints(std::vector<cv::Point2f>& pts);        

        float correct_angle;                                        // 经过卡尔曼滤波得到角度
        float angle;                                                // 图像信息解算得到的角度

        double timestamp;                                           // 时间戳
        
        cv::Point2f points[4];                                      // 返回的四点

        cv::Point2f circle_center;                                  // 能量机关圆心 
        cv::Point2f armor_center;                                   // 装甲板中心


        // 传统识别使用
        float vane_short_side;                                      //  短边
        float vane_long_side;                                       //  长边

        cv::Point2f mc;                                             // 质心

        std::vector<cv::Point2f> candidates_points;                 // approxPDP返回点集
        // std::vector<cv::Point2f> angle_points;                   // 角度筛选返回点集
        
        cv::RotatedRect vane_rrect;                                 // 大扇叶的最小外接矩形
        cv::RotatedRect armor_rrect;                                // 装甲板最小外接矩形

        cv::Point2f r_direction;                                    // 半径方向，圆心指向装甲板
        cv::Point2f find_circle_center_direction;                   // 寻找能量机关中心的矢量

        // rmos_interfaces::msg::DebugRuneArmor toDebugMsg();

        // 深度学习使用
        cv::Point2f dl_points[4];
        cv::Point2f dl_armor_center;
        bool have_correct_points = false;                           // 确定返回关键点数量正确，即接收五个关键点
    protected:
        /**
         *  @brief  初始化参数4
         *  @param  hull 角点
         */
        void initRuneArmor(std::vector<cv::Point2f> hull);

        /**
         *  @brief  找到指向能量机关圆心的矢量
         */
        void findDirection();

        float calDistance(Point2f pt1, Point2f pt2);
        Point2f correct_points[4];                                  // 外轮廓的正确点序
    };

} // namespace rmos_base


#endif