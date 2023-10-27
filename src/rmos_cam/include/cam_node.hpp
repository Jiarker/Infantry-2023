//
// Created by Wang on 23-6-14.
//

#ifndef RMOS_CAM_NODE_HPP
#define RMOS_CAM_NODE_HPP


//std
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>

//other
#include <opencv2/core.hpp>

//ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

#include "../../Algorithm/include/Camera/camera_interfaces/camera_interface.hpp"
#include "../../Algorithm/include/Camera/daheng/daheng.hpp"
#include "../../Algorithm/include/Camera/virtual_cam/virtual_cam.hpp"

#include "rmos_interfaces/msg/mode.hpp"
#include "rmos_interfaces/msg/exp.hpp"

namespace rmos_cam
{
    class CamNode : public rclcpp::Node
    {
        public:
            CamNode(const std::string & node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
            {
                RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
            };

        protected:
            image_transport::CameraPublisher img_pub_;                // 信息发布
            sensor_msgs::msg::CameraInfo camera_info_msg_;            // 相机消息
            sensor_msgs::msg::Image::SharedPtr image_msg_;
            rmos_interfaces::msg::Exp exp_msg;
            cv::Mat image_;
            std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;

            uint32_t frame_id_ = 0;                                   // 帧计数器
    };

    class DahengCamNode : public virtual CamNode
    {
    public:
        DahengCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~DahengCamNode();
    protected:


    /**
     *  @brief  设置模式
     */
    void setMode(int mode);

    void JudgeReset();

    void autoExpChange();

    // exp
    int normal_Exposure = 2500;
    int normal_Gamma = 9;
    
    int rune_Exposure = 1200;
    int rune_Gamma = 6;
    
    int auto_exp_change = 0;
    int max_exp = 5000;
    int min_exp = 300; 
    int change_num = 0;

    /*mode*/
    base::Mode mode_ = base::Mode::NORMAL;
    base::Mode previous_mode_ = base::Mode::NORMAL;
    int mode = 0;

    // 模式切换计数器
    int mode_num = 0;

    rclcpp::Publisher<rmos_interfaces::msg::Exp>:: SharedPtr exp_pub_;
    rclcpp::Subscription<rmos_interfaces::msg::Mode>::SharedPtr mode_sub_;
    std::shared_ptr<camera::DahengCam> cam_dev_;
    std::thread capture_thread_;                    // 采图线程

    };
    class VirtualCamNode : public virtual CamNode
    {
    public:
        VirtualCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~VirtualCamNode();
    protected:
        std::shared_ptr<camera::VirtualCam> virtual_dev_;
        std::thread capture_thread_;                    // 采图线程

    };



} // namespace rmos_cam



#endif //RMOS_CAM_NODE_HPP
