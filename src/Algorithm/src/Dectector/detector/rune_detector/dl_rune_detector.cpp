#include "Dectector/detector/rune_detector/rune_detector.hpp"


namespace detector
{
    DlRuneDetector::DlRuneDetector()
    {

        rt.init("./src/Algorithm/AIOF-IP/config/rtc_rm_v7_rune.yaml");
        std::cout<<rt.init_messages()<<std::endl;
        
        this->state = base::TrackState::LOST;

        cv::FileStorage fs("./src/Algorithm/configure/Detector/detector/rune_detector/Rune.xml", cv::FileStorage::READ);

        if(!fs.isOpened())
        {
            std::cout<<"open rune detect param fail"<<std::endl;
        }

        fs["show_R"] >>  param.show_R;

        fs["blue_brightness_thresh"] >>  param.blue_brightness_thresh;
        fs["blue_color_thresh"] >> param.blue_color_thresh;
        fs["red_brightness_thresh"] >>  param.red_brightness_thresh;
        fs["red_color_thresh"] >>  param.red_color_thresh;
        fs["blue_red_diff"] >>  param.blue_red_diff;
     
        fs["circle_center_conf"] >>  param.circle_center_conf;
        fs["acvitate_conf"] >>  param.acvitate_conf;
        fs["no_activate_conf"] >>  param.no_activate_conf;
        fs["circle_center_roi_width"] >>  param.circle_center_roi_width;

        fs["max_diff_distance_ratio"] >>  param.max_diff_distance_ratio;

        fs.release();
    }

    DlRuneDetector::~DlRuneDetector(){}

    bool DlRuneDetector::setEnemyColor(int enemy_color)
    {
        if (enemy_color == 0)
        {
            this->enemy_color_ = base::Color::RED;
            return true;
        }
        else if (enemy_color == 1)
        {
            this->enemy_color_ = base::Color::BLUE;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DlRuneDetector::DlRuneDetect(Mat& image,base::RuneArmor& target_rune_armor)
    {
        // 设置网络模型            
        std::vector<DetectResultList> outputs;
        std::vector<cv::Mat> inputs;
        std::vector<cv::Mat> result_mats;
        outputs.clear();
        inputs.clear();
        result_mats.clear();
        inputs.push_back(image);
        rt.run(inputs, outputs);
        std::vector<DetectResult> output = outputs[0];

        image.copyTo(src);
        DlPreDeal(image);

        drawDlMessage(image, output);

        FilterDetectResult(output);

        if(classifer(output))
        {
            DlDetectCircleCenter();
            this->target.circle_center = circle_center;
            
            DlDetectRuneArmor();

            drawTargetMessage(image);

            setFoundState();

            target_rune_armor = this->target;
            last_target = this->target;

            return true;
        }

        if (lost_times < 50 && state != base::TrackState::LOST)
        {
            state = base::TrackState::TEMP_LOST;
            lost_times++;
            std::cout << "Lost Times: " << lost_times << std::endl;
        }
        else
        {
            lost_times = 0;
            state = base::TrackState::LOST;
        }

        return false;
    }

    bool DlRuneDetector::ifOldArmor()
    {
        // // 上一个符为空，即本次识别到的符为第一帧
        // if (last_target.in_vane.rrect.boundingRect().empty())   
        //     return false;

        // 通过距离判断是否为旧符
        float max_diff_distance_ratio= calDistance(target.armor_center, last_target.armor_center) / calDistance(target.armor_center, target.circle_center);
        if (max_diff_distance_ratio > param.max_diff_distance_ratio)
        {
            cout<<"vane is changing!"<<endl;
            this->id++;
            return false;
        }

        return true;
    }

    void DlRuneDetector::setFoundState()
    {
        // the first for finding
        if(state == base::TrackState::LOST)
            state = base::TrackState::DETECTING;
        
        else
        {
            // the vane is changing
            if(!ifOldArmor())
                state = base::TrackState::DETECTING;
            else
            {
                if(lost_times < 5)
                    state = base::TrackState::TRACKING;
                else
                    state = base::TrackState::DETECTING;    
            } 
        }

        lost_times = 0;
    }

    void DlRuneDetector::drawDlMessage(Mat& image, std::vector<DetectResult> output)
    {
        for(int i = 0; i < output.size(); i++)
        {
            int point_num = output[i].points.size();
            for(int j = 0; j < point_num; j++)
            {
                cv::circle(image, output[i].points[j], 2, Scalar(255,255,0), -1);
                cv::line(image, output[i].points[j], output[i].points[(j+1)%point_num], Scalar(0, 100, 50));
            }
            std::string txt = "id:" + std::to_string(output[i].id) + " conf:" + std::to_string(output[i].conf);
            cv::putText(image, txt, output[i].bbox.br(), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                    cv::Scalar(0, 255, 0), 1.8);        
        }        
    }

    void DlRuneDetector::drawTargetMessage(Mat& image)
    {
        circle(image, this->circle_center, 4, Scalar(255,0,0), -1);
        circle(image, this->target.armor_center, 4, Scalar(127,127,127), -1);
        for(int i = 0; i < 4; i++)
            circle(image, this->target.points[i], 6, Scalar(127,127,127), -1);
    }

    bool DlRuneDetector::FilterDetectResult(std::vector<DetectResult>& output)
    {
        int vane_red_num = 0;
        int vane_blue_num = 0;
        // 初步阈值筛选，同时确定扇叶颜色
        for(auto iter = output.begin(); iter != output.end();)
        {
            // 扇叶初筛选
            if((*iter).id == 0 || (*iter).id == 3)
            {
                if(iter->conf < this->param.circle_center_conf)
                {
                    output.erase(iter);
                    continue;
                }
            }
            if((*iter).id == 1 || (*iter).id == 4)
            {
                if((*iter).conf < this->param.no_activate_conf)
                {
                    output.erase(iter);
                    continue;
                }
            }
            if((*iter).id == 2 || (*iter).id == 5)
            {
                if((*iter).conf < this->param.acvitate_conf)
                {
                    output.erase(iter);
                    continue;
                }
            }

            // 判断扇叶颜色
            if((*iter).id < 3) // 0-2为蓝色扇叶或圆心
                vane_blue_num++;
            else // 3-5为红色扇叶或圆心
                vane_red_num++;
            ++iter;
        }

        if(output.size() == 0)
            return false;
        
        if(this->enemy_color_ == base::Color::BLUE)
            this->vane_color = base::Color::RED;
        else
            this->vane_color = base::Color::BLUE;            
        
        return true;
    }

    bool DlRuneDetector::classifer(std::vector<DetectResult> output)
    {
        if(output.size() == 0)
            return false;        

        // 大符颜色与敌方颜色相符，则为打符;否则为反打符
        // 打符模式，返回深度学习解算的待激活扇叶与对应圆心(本程序已写死)
        bool find_no_activate = (this->vane_color != this->enemy_color_);
        bool find_blue = (this->vane_color == base::Color::BLUE);
        bool find_target = false;
        double last_target_conf = 0.0;
        double last_circle_center_conf = 0.0;
        double min_distance = DBL_MAX;
        for(int i = 0; i < output.size(); i++)
        {
            if(find_blue)
            {
                if(output[i].id == 0 && output[i].conf > last_circle_center_conf)
                {
                    this->dl_circle = output[i];
                    last_circle_center_conf = output[i].conf;
                }

                if(find_no_activate && output[i].id == 1 && output[i].conf > last_target_conf)
                {
                    this->target = base::RuneArmor(output[i].points);
                    last_target_conf = output[i].conf; 
                    find_target = true;        
                }       
                
                else if(!find_no_activate && output[i].id == 2)
                {
                    // 第一次发现目标
                    if(this->state == base::TrackState::LOST)
                    {
                        this->target = base::RuneArmor(output[i].points);          
                    } 
                    else
                    {
                        base::RuneArmor temp_rune_armor = base::RuneArmor(output[i].points);
                        double temp_distance = calDistance(temp_rune_armor.dl_armor_center, this->last_target.dl_armor_center); 
                        if(temp_distance < min_distance)
                        {
                            this->target = temp_rune_armor;  
                            min_distance = temp_distance;                         
                        }
                    } 
                    find_target = true; 
                }
            }

            else
            {
                if(output[i].id == 3 && output[i].conf > last_circle_center_conf)
                {
                    this->dl_circle= output[i];
                    last_circle_center_conf = output[i].conf;
                }

                if(find_no_activate && output[i].id == 4 && output[i].conf > last_target_conf)
                {
                    this->target = base::RuneArmor(output[i].points);
                    last_target_conf = output[i].conf; 
                    find_target = true;        
                }       
                
                else if(!find_no_activate && output[i].id == 5)
                {
                    // 第一次发现目标
                    if(this->state == base::TrackState::LOST)
                    {
                        this->target = base::RuneArmor(output[i].points);        
                    } 
                    else
                    {
                        base::RuneArmor temp_rune_armor = base::RuneArmor(output[i].points);
                        double temp_distance = calDistance(temp_rune_armor.dl_armor_center, this->last_target.dl_armor_center); 
                        if(temp_distance < min_distance)
                        {
                            this->target = temp_rune_armor;  
                            min_distance = temp_distance;                         
                        }
                    } 
                    find_target = true;  
                }
            }
        }
        // 若返回关键点点数量不等于5,即有关键点缺损，则返回false
        if(!this->target.have_correct_points)
            return false;

        return find_target;
    }

    void DlRuneDetector::DlPreDeal(Mat frame)
    {
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        std::vector<Mat> bgr;
        split(frame, bgr);
        Mat gray_bin, color_bin;

        if (this->vane_color == base::Color::RED)
        {
            threshold(gray, gray_bin, param.red_brightness_thresh, 255, THRESH_BINARY);
            subtract(bgr[2], bgr[0], color_bin);
            threshold(color_bin, color_bin, param.red_color_thresh, 255, THRESH_BINARY);
        }
        else
        {
            threshold(gray, gray_bin, param.blue_brightness_thresh, 255, THRESH_BINARY);
            subtract(bgr[0], bgr[2], color_bin);
            threshold(color_bin, color_bin, param.blue_color_thresh, 255, THRESH_BINARY);
        }

        this->bin = gray_bin & color_bin;
        // Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::Mat element = (cv::Mat_<float>(3, 3) << 1, 1, 1,
        //                                             1, 1, 1,	
        //                                             1, 1, 1);

        // cv::Mat element = (cv::Mat_<float>(5, 5) << 1, 1, 1, 1, 1,
        //                                     1, 1, 1, 1, 1,	
        //                                     1, 1, 1, 1, 1,
        //                                     1, 1, 1, 1, 1,
        //                                     1, 1, 1, 1, 1);              
    }

    bool DlRuneDetector::DlDetectCircleCenter()
    {
        Point2f all_points = Point2f(0.0,0.0);
        int points_num = dl_circle.points.size();
        if(points_num < 5)
        {
            cout<<"not enough points for circle center!"<<endl;
            return false;
        }
        for(int i = 0; i < points_num; i++)
            all_points += Point2f(this->dl_circle.points[i]);
        Point2f dl_circle_center = all_points / points_num; 
        float R_roi_width = this->param.circle_center_roi_width;
        // 判断roi越界
        if(dl_circle_center.x - R_roi_width < 0 || dl_circle_center.x + R_roi_width > bin.cols || dl_circle_center.y - R_roi_width < 0 || dl_circle_center.y + R_roi_width > bin.rows)
        {
            cout<<"roi越界"<<endl;
            this->circle_center = dl_circle_center;
            return true;
        }
        cv::Rect2f R_roi_rect = cv::Rect2f(dl_circle_center.x - R_roi_width, dl_circle_center.y - R_roi_width, 2 * R_roi_width, 2 * R_roi_width);
        R_roi_rect &= cv::Rect2f(cv::Point2f(0, 0), cv::Point2f(bin.cols, bin.rows));
        this->R_bin = bin(R_roi_rect);
        
        if(param.show_R)
        {
            cv::imshow("R_Bin", this->R_bin);
            waitKey(1);
        }

        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(R_bin, contours, hierarchy, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point2f(R_roi_rect.x, R_roi_rect.y));
        if (contours.size() <= 0)
        {
            cout<<"未检测到轮廓"<<endl;
            this->circle_center = dl_circle_center;
            return true;
        }

        cv::RotatedRect R_rect;
        double max_area = 0.0;
        for(int i = 0; i < contours.size(); i++)
        {
            float area = contourArea(contours[i]);     
            if(area > max_area)
            {
                R_rect = minAreaRect(contours[i]);
                max_area = area;
            }       
        }

        this->circle_center = R_rect.center;

        return true;
    }

    bool DlRuneDetector::DlDetectRuneArmor()
    {
        Point2f all_points = Point2f(0.0, 0.0);
        for(int i = 0; i < 4; i++)
        {
            bool is_show_point = false;
            if(i == 0)
                is_show_point = true;

            this->target.points[i] = this->target.dl_points[i];
            all_points += this->target.points[i];
        }
        this->target.armor_center = all_points / 4;

        target.r_direction = target.armor_center - target.circle_center;
        target.angle = atan2(target.r_direction.y, target.r_direction.x);

        // 角度转换 
        if(target.angle < 0)
            target.angle += 2 * CV_PI;

        return true;
    }
}