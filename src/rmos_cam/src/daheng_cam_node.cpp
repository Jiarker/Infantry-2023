//
// Created by Wang on 23-6-14.
//


//std
#include <chrono>
#include <sstream>

//ros
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../include/cam_node.hpp"

namespace rmos_cam
{
    DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options) : CamNode("daheng_camera", options)
    {
        // cam dev
        cam_dev_ = std::make_shared<camera::DahengCam>();

        // parameter
        int Width, Height, Exposure, RGain, GGain, BGain, Gamma, Fps;
        int AutoExposure, AutoWhiteBalance;

        cv::FileStorage fs("./rmos_bringup/configure/daheng_camera.xml", cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Open daheng_camera.xml fail!");
            exit(0);
        }

        fs["width"] >> Width;
        fs["height"] >> Height;
        fs["exposure"] >> Exposure;
        fs["rgain"] >> RGain;
        fs["ggain"] >> GGain;
        fs["bgain"] >> BGain;
        fs["fps"] >> Fps;
        fs["gain"] >> Gamma;

        fs["rune_exposure"] >> this->rune_Exposure;
        fs["rune_gain"] >> this->rune_Gamma;

        fs["auto_exp_change"] >> this->auto_exp_change;
        fs["max_exp"] >> this->max_exp;
        fs["min_exp"] >> this->min_exp;

        fs.release();

        this->normal_Exposure = Exposure;
        this->normal_Gamma = Gamma;

        std::cout << "rune_Gamma:" << rune_Gamma << std::endl;
        std::cout << "rune_Exposure:" << rune_Exposure << std::endl;

        std::cout << "Gamma:" << Gamma << std::endl;
        std::cout << "Exposure:" << Exposure << std::endl;

        // set paramter
        cam_dev_->set_parameter(camera::CamParamType::Height, Height);
        cam_dev_->set_parameter(camera::CamParamType::Width, Width);
        cam_dev_->set_parameter(camera::CamParamType::AutoExposure, AutoExposure);
        cam_dev_->set_parameter(camera::CamParamType::Exposure, Exposure);
        cam_dev_->set_parameter(camera::CamParamType::AutoWhiteBalance, AutoWhiteBalance);
        cam_dev_->set_parameter(camera::CamParamType::RGain, RGain);
        cam_dev_->set_parameter(camera::CamParamType::GGain, GGain);
        cam_dev_->set_parameter(camera::CamParamType::BGain, BGain);
        cam_dev_->set_parameter(camera::CamParamType::Gamma, Gamma);
        cam_dev_->set_parameter(camera::CamParamType::Fps, Fps);

        cam_dev_->open();

        img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);

        // load camera_info
        cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "DahengCam");
        auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_bringup");
        auto yaml_path = "file://" + pkg_path + "/configure/daheng_cam_info.yaml";
        if (!cam_info_manager_->loadCameraInfo(yaml_path))
        {
            RCLCPP_WARN(this->get_logger(), "Load Camera Info Fail!");
        }
        else
        {
            camera_info_msg_ = cam_info_manager_->getCameraInfo();
        }

        // exp publish
        this->exp_pub_ = this->create_publisher<rmos_interfaces::msg::Exp>("/exp_info", rclcpp::SensorDataQoS());

        // get mode
        this->mode_sub_ = this->create_subscription<rmos_interfaces::msg::Mode>
                ("/mode_info", rclcpp::SensorDataQoS(), [this](rmos_interfaces::msg::Mode::ConstSharedPtr mode_msg)
                {
                    // RCLCPP_INFO(this->get_logger(), "mode is %d", (*mode_msg).mode);
                    this->mode = (*mode_msg).mode;
                    setMode(mode);
                });

        capture_thread_ = std::thread{[this]() -> void
                                      {
                                          while (rclcpp::ok())
                                          {

                                              // 判断是否需要模式切换
                                              // this->mode_ = rmos_base::Mode::RUNE;
                                              JudgeReset();

                                              if (!cam_dev_->is_open())
                                              {
                                                  exit(0);
                                              }
                                              

                                              if (cam_dev_->grab_image(image_))
                                              {
                                                  image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",image_).toImageMsg();
                                                  (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now();
                                                  (*image_msg_).header.frame_id = "camera";
                                                  camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;

                                                  if(this->auto_exp_change)
                                                    autoExpChange();

                                                  exp_msg.exp = cam_dev_->params_[camera::CamParamType::Exposure];
                                                  exp_pub_->publish(exp_msg);
                                                  img_pub_.publish(*image_msg_, camera_info_msg_);

                                              }
                                              else
                                              {
                                                  std::cout << cam_dev_->error_message() << std::endl;
                                                  exit(0);
                                              }
                                          }
                                      }};
    }

    void DahengCamNode::setMode(int mode)
    {
        if(mode == 1)
            this->mode_ = base::Mode::NORMAL_RUNE;
        else if(mode == 2)
            this->mode_ = base::Mode::RUNE;
        else if(mode == 0)
            this->mode_ = base::Mode::NORMAL;
        else 
            this->mode_ = base::Mode::WAIT;
    }

    void DahengCamNode::JudgeReset()
    {
        if(this->mode_ != this->previous_mode_ && this->mode_ != base::Mode::WAIT)
        {
            // std::cout<<"mode_num:"<<mode_num<<std::endl;
            // this->mode_num++;   
            // if(this->mode_num < 0)
            // {
            //     return;
            // }
            // this->mode_num = 0;
            // std::cout<<"change mode"<<std::endl;
            if(this->mode_ == base::Mode::NORMAL)
                cam_dev_->changeExp(this->normal_Exposure);
            else
                cam_dev_->changeExp(this->rune_Exposure);   
            this->previous_mode_ = this->mode_;             
        }
        else
            this->mode_num = 0;

    }

    void DahengCamNode::autoExpChange()
    {
        int now_exp = cam_dev_->params_[camera::CamParamType::Exposure];
        if(this->change_num < 5)
        {
            change_num++;
            return;
        }
        change_num = 0;
        now_exp += 100;
        if(now_exp < this->min_exp || now_exp > this->max_exp)
            now_exp = this->min_exp;
        cam_dev_->changeExp(now_exp);
        std::cout<<"exp:"<< cam_dev_->params_[camera::CamParamType::Exposure] << std::endl;
    }

    DahengCamNode::~DahengCamNode()
    {
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        cam_dev_->close();
        RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }
} // namespace rmos_cam


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::DahengCamNode)






