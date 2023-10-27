#ifndef RMOS_DETECTOR__RUNE_DETECTOR_HPP_
#define RMOS_DETECTOR__RUNE_DETECTOR_HPP_

#include "Base/rune_armor.hpp"
#include "../../../../AIOF-IP/include/Common.hpp"
#include "../../../../AIOF-IP/include/Runtime.hpp"

namespace detector
{
    using namespace cv;
    using namespace std;

    typedef struct RuneParam
    {
    public:
        int show_R;
 
        //图像参数
        float blue_brightness_thresh;
        float blue_color_thresh;
        float red_brightness_thresh;
        float red_color_thresh;
        float blue_red_diff;   
        
        float circle_center_conf;                   // 能量机关圆心阈值
        float no_activate_conf;                     // 未激活扇叶阈值
        float acvitate_conf;                        // 已激活扇叶阈值
        float circle_center_roi_width;              // 中心R标roi宽

        float max_diff_distance_ratio;              // judge for old vane
        
        // 拟合使用参数
        float delay_time;                           // 预测延迟
        int save_txt;                             // 保存txt文件用来绘图

    } RuneParam; 


    class DlRuneDetector{
    
    public:
        cv::Mat src;
        cv::Mat bin;
        cv::Mat R_bin;
        cv::Mat gray;

        base::TrackState state = base::TrackState::LOST;
        base::RuneArmor target;
        base::RuneArmor last_target;
        std::vector<cv::Point2f> nextPos;
    
        int id = 0;                                     // 扇叶识别次序
        RuneParam param;
        base::Color enemy_color_{base::BLUE};

        /**
         *  @brief  构造函数,初始化yolov7模型与RuneParam内的参数
         */
        DlRuneDetector();

        ~DlRuneDetector(); 

        /**
         *  @brief  接收通信节点回传参数,进而设置敌方颜色参数
         *  @param  color 通信节点发送的经转换后敌方颜色参数
         */
        bool setEnemyColor(int color);

        /**
         *  @brief  大符识别主调函数
         *  @param  image  从相机节点接收的图片
         *  @param  target_rune_armor  识别得到的当前待击打扇叶详细数据
         */
        bool DlRuneDetect(Mat& image,base::RuneArmor& target_rune_armor);   

    protected:
        /*YOLO constructer*/
        Runtime rt;

        int lost_times = 0; // 丢失目标帧数
       
        Point2f circle_center;    
        DetectResult dl_circle;
        base::Color vane_color = base::Color::BLUE;
        Point2f dl_points[4];

        /**
         *  @brief  通过当前扇叶与上次识别的扇叶靶心距离判断扇叶切换情况
         */
        bool ifOldArmor();

        /**
         *  @brief  找到装甲板后的跟踪状态设置
         */
        void setFoundState();

        /**
         * @brief 将神经网络模型解算结果绘制在原始图片上
         * @param image 原始图片
         * @param output 网络返回结果
         */    
        void drawDlMessage(Mat& image, std::vector<DetectResult> output);
        
        /**
         * @brief 将识别节点解算结果绘制在原始图片上
         * @param image 原始图片
        */
        void drawTargetMessage(Mat& image);

        /**
         *  @brief  对网络得出的结果进行筛选,选择大于阈值的已激活扇叶\未激活扇叶\能量机关圆心,并依据敌方颜色判断扇叶颜色,为后期预处理做准备
         *  @param  output 网络返回结果
        */
        bool FilterDetectResult(std::vector<DetectResult>& output);

        /**
         * @brief  对网络结果分类，并判断是否有待击打扇叶
         * @param output 网络返回结果 
        */
        bool classifer(std::vector<DetectResult> output);

        /**
         * @brief 图像预处理
         * @param frame 图像
         * @result bin 二值图
        */
        void DlPreDeal(Mat frame);

        /**
         * @brief 找到能量机关圆心
         * @param bin 二值图
        */
        bool DlDetectCircleCenter();
        
        /**
         * @brief 处理识别数据,为后期拟合做准备
         * @result armor_center 装甲板中心
         * @result angle 图像信息解算得到的角度
         * @result r_direction 半径方向,圆心指向装甲板
        */
        bool DlDetectRuneArmor();

        /**
         * @brief 计算两点距离
        */
        float calDistance(cv::Point2f pt1, cv::Point2f pt2)
        {
            cv::Point2f dis = pt1 - pt2;
            return sqrt(pow(dis.x,2)+pow(dis.y,2));
        };
    };


}// namespace detector

#endif