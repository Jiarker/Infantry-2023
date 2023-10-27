#include "Dectector/fittool/Fitting.h"

namespace detector
{

    bool Judgement::Judge(double& judge_speed, bool is_direction_inited, bool is_clockwise)
    {
        // 速度初筛选

        // 方向未初始化
        if(!is_direction_inited && fabs(judge_speed) > 2.7)
        {
            solveBadData();
            return false;
        }
        
        else
        {
            // 顺时针情况
            if(is_clockwise)
            {
                if(judge_speed > 2.7 || judge_speed < -0.4)
                // if(judge_speed < -1.8)
                {
                    solveBadData();
                    return false;
                }
            }

            // 逆时针情况
            else
            {
                if(judge_speed < -2.7 || judge_speed > 0.4) 
                // if(judge_speed > 1.8)
                {
                    solveBadData();
                    return false;
                }
            }
        }

        // 3o筛选
        
        if(speedJudge.size() < 15)
        {
            speedJudge.push_back(judge_speed);
            judge_clear_num = 0;
            return true;
        }

        getN();
        getMean();
        getVariance();

        if(judge_speed > mean + 3*standard_deviation || judge_speed < mean - 3*standard_deviation)
        {
            solveBadData();
            return false;
        }
        
        // 确认为正确值

        speedJudge.push_back(judge_speed);

        while (speedJudge.size() > 15)
            speedJudge.erase(speedJudge.begin());
        
        judge_clear_num = 0;
        return true;

    }

    void Judgement::resetJudge()
    {
        judge_clear_num = 0;
        speedJudge.clear();
    }

    void Judgement::getN()
    {
        this->n = speedJudge.size();
    }

    void Judgement::getMean()
    {
        double sum = 0;
        for(int i = 0; i < n; i++)
            sum += speedJudge[i];
        this->mean = sum / n;
    }

    void Judgement::getVariance()
    {
        double square_sum = 0;
        for(int i = 0; i < n; i++)
            square_sum += pow(speedJudge[i], 2);
        this->variance = (square_sum - n*pow(mean,2)) / n;
        // 防止标准差为0,设置标准差恒为0.1
        if(this->variance < 0.01)
            this->variance = 0.01;
        if(this->variance > 0.04)
            this->variance = 0.04;
            
        this->standard_deviation = pow(this->variance, 0.5);
    }

    void Judgement::solveBadData()
    {
        judge_clear_num++;
        if(judge_clear_num > 3)
        {
            judge_clear_num = 0;
            speedJudge.clear();
        }    
    }

    /*-----------FitTool-----------*/
    FitTool::FitTool()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Detector/detector/rune_detector/Rune.xml", cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            std::cout<<"open rune detect param fail"<<std::endl;
        }
        fs["delay_time"] >> DELAY_TIME;
        fs["save_txt"] >> save_txt;
        fs["print_result"] >> print_result;
        fs.release();
    }

    bool FitTool::run(base::RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state, base::Mode rune_mode)
    {
        bool result = false;
        switch (rune_mode){
            case base::Mode::NORMAL_RUNE:
                return runNormalRune(armor_1, nextPosition, armor_state);

            case base::Mode::RUNE:
                return runRune(armor_1, nextPosition, armor_state);
        
            default:
                return false;
        }

    }

    void FitTool::clearData()
    {
        cout << "Clear Fitting Data!" << endl;
        fitting_data.clear();
        armor_buffer.clear();
        judge.resetJudge();
        is_direction_inited = false;
    }

    bool FitTool::runRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state)
    {
        if (!processDataState(armor_1, armor_state))
            return false;
        
        initDirection();
        if (is_direction_inited)
        {
            fittingCurve();
            if(print_result)
                printResult();
            
            if (is_Inited)
            {
                if(save_txt)
                    dataWriting(armor_1.timestamp);

                nextPosition.clear();
                vector<cv::Point2f> pts;
                armor_1.getPoints(pts);
                double delta = deltaAngle(armor_1.timestamp);
                this->delta_angle = delta;
                this->delta_speed = deltaSpeed(armor_1.timestamp);
                this->now_state = SpeedTime(calSpeed(armor_1.timestamp), armor_1.timestamp);

                for (int i = 0; i < 4; i++)
                {
                    nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
                }
                return true;
            }
            else
            {
                nextPosition.clear();
                vector<cv::Point2f> pts;
                armor_1.getPoints(pts);
                double delta = CV_PI / 3 * DELAY_TIME;

                if(!fitting_data.empty())
                {
                    this->delta_angle = delta;
                    this->delta_speed = deltaSpeed(armor_1.timestamp);
                    this->now_state = SpeedTime(calSpeed(armor_1.timestamp), armor_1.timestamp);
                }

                for (int i = 0; i < 4; i++)
                {
                    nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
                }
                return true; 
            }
        }
        return false;
    }

    bool FitTool::runNormalRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state)
    {
        switch (armor_state)
        {
        case base::TrackState::DETECTING:
            lost_time = 0;
            clearArmorBuffer();
            if (armor_1.timestamp <= 0.01)
                return false;
            armor_buffer.push_back(armor_1);
            break;
        case base::TrackState::TRACKING:
            lost_time = 0;
            if (armor_1.timestamp <= 0.01)
                return false;
            armor_buffer.push_back(armor_1);
            if (armor_buffer.size() >= DN + 1)
            {
                double temp_speed = calAngleSpeed(armor_1, armor_buffer[armor_buffer.size() - 1 - DN]);

                // if(Judge(temp_speed))
                {
                    SpeedTime temp_state = SpeedTime(temp_speed, (armor_1.timestamp + armor_buffer[armor_buffer.size() - 1 - DN].timestamp) / 2);
                    pushFittingData(temp_state);
                    this->now_temp_state = temp_state;
                }

                while (armor_buffer.size() > DN + 1)
                    armor_buffer.erase(armor_buffer.begin());
                while (fitting_data.size() > 200)
                    fitting_data.erase(fitting_data.begin());
            }
            break;
        case TrackState::LOST:
            clearData();
            return false;
        default:
            return false;
            break;
        }
        initDirection();

        if (is_direction_inited)
        {
            nextPosition.clear();
            vector<cv::Point2f> pts;
            armor_1.getPoints(pts);
            double delta = CV_PI / 3 * DELAY_TIME;
            for (int i = 0; i < 4; i++)
            {
                nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
            }
            return true; 
        }
        return false;
    }

    cv::Point2f FitTool::calNextPosition(cv::Point2f point, cv::Point2f org, float rotate_angle)
    {
        double radius = calDistance(point, org);
        cv::Point2f relative_point = point - org;                                         // 相对坐标
        double relative_angle = atan2(relative_point.y, relative_point.x);                // 与圆心所成角度
        double next_angle;

        if (is_clockwise) // 顺时针运动
        {
            next_angle = relative_angle + rotate_angle;
            if (next_angle > CV_PI)
                next_angle -= 2.0 * CV_PI;
        }
        else
        {
            next_angle = relative_angle - rotate_angle;
            if (next_angle < - CV_PI)
                next_angle += 2.0 * CV_PI;
        }

        return cv::Point2f(cos(next_angle) * radius, sin(next_angle) * radius) + org;
    }

    bool FitTool::processDataState(RuneArmor armor_1, TrackState armor_state)
    {
        switch (armor_state)
        {
        case base::TrackState::DETECTING:
            clearArmorBuffer();
            if (armor_1.timestamp <= 0.01)
                return false;
            armor_buffer.push_back(armor_1);
            break;

        case base::TrackState::TRACKING:
            if (armor_1.timestamp <= 0.01)
                return false;

            armor_buffer.push_back(armor_1);
            while (armor_buffer.size() > 1 + DN)
            {
                double delta_time = armor_1.timestamp - armor_buffer[0].timestamp;
                if (delta_time > 0.2)
                    armor_buffer.erase(armor_buffer.begin());
                else if (delta_time > 0.005)
                {
                    double temp_speed = calAngleSpeed(armor_1, armor_buffer[armor_buffer.size() - 1 - DN]);
                    if(judge.Judge(temp_speed, is_direction_inited, is_clockwise))
                    {
                        SpeedTime temp_state = SpeedTime(temp_speed, (armor_1.timestamp + armor_buffer[armor_buffer.size() - 1 - DN].timestamp) / 2);
                        pushFittingData(temp_state);
                        this->now_temp_state = temp_state;
                    }
                    break;
                }
                else
                    break;
            }

            break;

        case TrackState::LOST:
            armor_buffer.clear();
            fitting_data.clear();
            judge.resetJudge();
            return false;
            break;
            
        default:
            return false;
            break;
        }
        while (fitting_data.size() > N)
            fitting_data.erase(fitting_data.begin());
        
        return true;
    }

    void FitTool::pushFittingData(SpeedTime new_data)
    {
        if (fitting_data.empty())
        {
            fitting_data.push_back(new_data);
            return;
        }
        SpeedTime flag_data = fitting_data[fitting_data.size() - 1];
        if ((double)new_data.time - (double)flag_data.time - DT < 0)
        {
            return;
        }

        double T = DT;
        double n = ((double)new_data.time - (double)flag_data.time) / T;

        if (n > 50)
        {
            clearData();
            return;
        }

        for (int i = 0; i < (int)n; i++)
        {
            double temp_T = T * (i + 1);
            double delta = (double)new_data.time - (double)flag_data.time - temp_T;
            SpeedTime temp = SpeedTime(new_data.angle_speed * (temp_T / (temp_T + delta)) + flag_data.angle_speed * (delta / (temp_T + delta)), flag_data.time + (double)temp_T);  
            fitting_data.push_back(temp); 
            this->now_correct_state.push_back(temp);  
        }
    }

    void FitTool::initDirection()
    {
        if (fitting_data.size() >= 50)
        {
            int clock = 0, clock_inv = 0;
            for (int i = 0; i < fitting_data.size(); i++)
            {
                if (fitting_data[i].angle_speed > 0)
                    clock++;
                else
                    clock_inv++;
            }
            is_direction_inited = true;
            is_clockwise = clock > clock_inv;
        }
    }

    void FitTool::clearArmorBuffer()
    {
        armor_buffer.clear();
        judge.resetJudge();
    }

    double FitTool::calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2)
    {
        double time_diff = (double)(armor_1.timestamp - armor_2.timestamp);

        if(time_diff < 0.005)
            time_diff += 0.005;

        double angle_diff = armor_1.angle - armor_2.angle;
        if (armor_1.angle < -CV_PI / 2.0 && armor_2.angle > CV_PI / 2.0)
            angle_diff = angle_diff + CV_PI * 2.0;
        else if (armor_1.angle > CV_PI / 2.0 && armor_2.angle < -CV_PI / 2.0)
            angle_diff = angle_diff - CV_PI * 2.0;
        
        return angle_diff / time_diff; // 转换单位
    }

    /*---------------拟合函数-----------------*/
    void FitTool::fittingCurve()
    {
        if (fitting_data.empty())   return;
        if (fitting_data[fitting_data.size() - 1].time - fitting_data[0].time >= (N - 1) * DT - 1)
        {
            fitting_a_w();
            if (isnan(_a))
            {
                cout << "A nan" << endl;
                _a = 0.9;
                _w = 1.9;
                t_0 = 0;
                clearData();
                return;
            }
            fitting_t();
            is_Inited = true;
        }
    }

    void FitTool::fitting_a_w()
    {
        int n_min = 1.884 / (2 * M_PI) * N;
        int n_max = 2.0 / (2 * M_PI) * N + 1;

        double max_i = n_min;
        double max_value = get_F(n_min, N), value = 0.0;
        for (int i = n_min + 1; i < n_max; i++)
        {
            value = get_F(i, N);
            if (value > max_value)
            {
                max_i = (double)i;
                max_value = value;
            }
        }

        _w = max_i / (double)N * 2.0 * M_PI;
        _a = max_value / N * 2;
        if (_a > 1.045) _a = 1.045;
        else if (_a < 0.780) _a = 0.780;

    }

    void FitTool::fitting_t()
    {
        double max_value = 0.0, value = 0.0;
        int max_i = 0;
        for (int i = 0; i < T0_N + 1; i++)
        {
            value = get_integral((double)i * MAX_T0 / T0_N);
            if (value > max_value)
            {
                max_i = i;
                max_value = value;
            }
        }
        t_0 = (double)max_i * MAX_T0 / T0_N;
        start_time = fitting_data[0].time;
    }

    double FitTool::get_F(int n, int _N)
    {
        double c = 0.0, s = 0.0;
        if (is_clockwise)
            for (int i = 0; i < fitting_data.size(); i++)
            {
                c += get_F_c(n, (fitting_data[i].angle_speed - (2.090 - _a)), i, N);
                s += get_F_s(n, (fitting_data[i].angle_speed - (2.090 - _a)), i, N);
            }
        else
            for (int i = 0; i < fitting_data.size(); i++)
            {
                c += get_F_c(n, (-fitting_data[i].angle_speed - (2.090 - _a)), i, N);
                s += get_F_s(n, (-fitting_data[i].angle_speed - (2.090 - _a)), i, N);
            }

        return sqrt(c * c + s * s);
    }

    double FitTool::get_integral(double t_)
    {
        double sum = 0;
        if (is_clockwise)
            for (int i = 0; i < fitting_data.size(); i++)
            {
                sum += sin((i * DT + t_) * _w) * (fitting_data[i].angle_speed - (2.090 - _a)) / _a;
            }
        else
            for (int i = 0; i < fitting_data.size(); i++)
            {
                sum += sin((i * DT + t_) * _w) * (-fitting_data[i].angle_speed - (2.090 - _a)) / _a;
            }

        return sum;
    }

    /*---------------data writing-----------------*/
    void FitTool::dataWriting(double timestamp)
    {
        if(!have_first_time)
        {
            first_time = timestamp;
            have_first_time = true;
            return;
        }
        string path = "./src/Algorithm/configure/Detector/fittool/buff_state/" + to_string(int(first_time)) + ".txt";
        buff_data.open(path, ios::app);
        buff_data << now_temp_state.angle_speed << " " << now_temp_state.time - first_time << " "
                << delta_angle << " " << delta_speed << " "
                << now_state.angle_speed << " " << now_state.time - first_time << " ";
        
        for(int i = 0; i < now_correct_state.size(); i++)
            buff_data << now_correct_state[i].angle_speed << " " << now_correct_state[i].time - first_time << " ";
        buff_data << "\n";
        buff_data.close();

        now_correct_state.clear();
        buff_data.close();
        
    }

    double FitTool::deltaAngle(double time)
    {
        double t = (double)(time - start_time);
        
        return (-_a / _w) * (cos(_w * (t + DELAY_TIME + t_0)) - cos(_w * (t + t_0))) + (2.090 - _a) * DELAY_TIME;
    }

    double FitTool::deltaSpeed(double time)
    {
        double t = (double)(time - start_time);
        return fabs(_a * sin(_w * (t + t_0)) - _a * sin(_w * (t + t_0 + DELAY_TIME)));
    }

    double FitTool::calSpeed(double time)
    {
        double t = (double)(time - start_time);
        return (_a * sin(_w * (t + t_0)) + 2.090 - _a);
    }

}
