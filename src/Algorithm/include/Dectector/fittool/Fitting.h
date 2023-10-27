/**
 * (a * sin(w * (x_ + t)) + 2.090 - a)
 */

#ifndef FITTING_H
#define FITTING_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "../../Base/rune_armor.hpp"
#include "../../Base/const.hpp"

namespace detector{

using namespace std;
using namespace cv;
using namespace base;

typedef struct SpeedTime
{
    double angle_speed; // y
    double time;      // x

    SpeedTime(double speed = 0.0, double t = 0)
    {
        angle_speed = speed;
        time = t;
    }
} SpeedTime;

class Judgement{
public:
    Judgement(){};
    ~Judgement() = default;

    /**
     * @brief 判断数据是否为坏值
     * @param judge_speed 待判断数据
     * @param is_derection_inited 能量机关方向是否初始化
     * @param is_clockwise 能量机关方向是否是顺时针 
    */
    bool Judge(double& judge_speed, bool is_direction_inited, bool is_clockwise);

    /**
     * @brief 判断器重置
    */
    void resetJudge();

private:
    vector<double> speedJudge;          // 判断器数据集
    double n;
    double mean;
    double variance;
    double standard_deviation;
    int judge_clear_num = 0;            // 坏值累加器,若大于3,则重置判断器

    /**
     * @brief 得到判断器内数据集个数
    */
    void getN();

    /**
     * @brief 计算数据集的平均值
    */
    void getMean();

    /**
     * @brief 计算方差
    */
    void getVariance();

    /**
     * @brief 处理坏值
    */
    void solveBadData();
};

class FitTool
{
public:
    vector<RuneArmor> armor_buffer;     // 存储一段时间内的未激活扇叶集合,用来计算速度
    vector<SpeedTime> fitting_data;     // 拟合数据集

    double DELAY_TIME = 0.50;           // 预测时间，单位：秒
    int save_txt;
    int print_result;

private:

    Judgement judge;

    double _a = 0.9;                    // 振幅 [0.780, 1.045]
    double _w = 1.9;                    // 频率 [1.884, 2.000]
    double t_0 = 0.0;                   // 初相位

    double MAX_T0 = 3.34;               // 最大周期
    double T0_N = 30;                   // 相位采样数
    double DT = 0.01;                   // 采样时间间隔，单位：秒
    double N = 400;                     // 角速度采样数

    int DN = 1;                         // 逐差法测速度间距

    double start_time;                  // 拟合数据集中的第一个时间戳
    bool is_Inited = false;             // 大符拟合是否初始化
    bool is_direction_inited = false;   // 能量机关旋转方向初始化
    bool is_clockwise;                  // 顺时针

    int lost_time = 0;

    // data writing
    ofstream buff_data;                 // 文件头
    double first_time = 0;              // 第一个进入拟合函数的时间戳,用来当作文件名称
    bool have_first_time = false;       // 判断是否出现第一个进入拟合函数的时间戳,进而判断是否需要新建txt文件 
    SpeedTime now_temp_state;           // 最新计算的未放入线性插值的SpeedTime
    vector<SpeedTime> now_correct_state;// 当前最新计算的放入线性插值之后的vector<SpeedTime>
    double delta_angle;                 // 预测的旋转角度
    double delta_speed;                 // 预测的速度变化
    SpeedTime now_state;                // 当前SpeedTime状态

public:
    /**
     * @brief 初始化参数
    */
    FitTool();

    /**
     *  @brief  打印结果
     */
    void printResult()
    {
        cout << "a: " << _a << "\tw: " << _w << "\tt: " << t_0 << endl;
    }
    
    ~FitTool() = default;

    /**
     *  @brief  封装API
     */
    bool run(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state, Mode rune_mode=Mode::RUNE);

protected:
    /**
     *  @brief  清空数据
     */
    void clearData();

    /**
     *  @brief  击打大符模式
     */
    bool runRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state);

    /**
     *  @brief  击打小符模式
     */ 
    bool runNormalRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state);

    /**
     *  @brief  根据旋转角度和半径计算下一点(装甲板四个角点)的像素位置
     *  @param  point   动点
     *  @param  org     原点
     *  @param  angle   旋转角度
     */
    cv::Point2f calNextPosition(cv::Point2f point, cv::Point2f org, float angle);

    /**
     *  @brief  根据状态处理数据
     *  @param  armor_1 处理完的装甲板
     *  @param  timestamp   原图像时间戳
     */
    bool processDataState(RuneArmor armor_1, TrackState armor_state);

    /**
     *  @brief  数据作插值处理
     */
    void pushFittingData(SpeedTime new_data);

    /**
     *  @brief  判断能量机关旋转方向
     */
    void initDirection();

    /**
     *  @brief  清空armor_buffer
     */
    void clearArmorBuffer();

    /**
     *  @brief  计算瞬时角速度 (armor_1 - armor_2)
     *  @param  armor_1 新目标
     *  @param  armor_2 老目标
     *  @return 角速度，单位 弧度/秒
     */
    double calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2);

    /**
     *  @brief  初步拟合数据
     */
    void fittingCurve();

    /**
     *  @brief  通过离散傅里叶变换求w
     */
    void fitting_a_w();

    /**
     *  @brief  拟合相位
     */
    void fitting_t();

    /**
     *  @brief  离散傅里叶获得正弦项值
     */
    double get_F_s(int n, double f_k, int k, int _N)
    {
        return f_k * sin(2.0 * M_PI * (double)n / (double)_N * (double)k * DT);
    }

    /**
     *  @brief  离散傅里叶获得余弦项值
     */
    double get_F_c(int n, double f_k, int k, int _N)
    {
        return f_k * cos(2.0 * M_PI * (double)n / (double)_N * (double)k * DT);
    }

    /**
     *  @brief 离散傅里叶获得第n项的值，规整化速度值
     *  @return 模的平方
     */
    double get_F(int n, int _N);

    /**
     *  @brief  求不同相位时的积分,规整化速度值
     */
    double get_integral(double t_);

    /**
     * @brief 计算两点距离
    */
    float calDistance(cv::Point2f pt1, cv::Point2f pt2)
    {
        cv::Point2f dis = pt1 - pt2;
        return sqrt(pow(dis.x,2)+pow(dis.y,2));
    };

    /**
     * @brief 将数据绘制于txt文件
     * @param timestamp 时间戳
    */
    void dataWriting(double timestamp);

    /**
     *  @brief  速度函数积分计算偏移角
     *  @param  time    装甲板时间戳
     *  @return 返回增大的角度，单位：弧度
     */
    double deltaAngle(double time);

    /**
     * @brief 计算当前时间与t0时刻时间的速度差
     * @param time 当前时间戳
    */
    double deltaSpeed(double time);

    /**
     * @brief 计算当前时间戳经公式计算后预测的速度
     * @param time 当前时间戳
    */
    double calSpeed(double time);
};

}

#endif