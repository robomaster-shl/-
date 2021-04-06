#ifndef FILTER_H
#define FILTER_H

#include<iostream>
#include</usr/local/include/eigen3/Eigen/Dense>
#include<stdio.h>
using namespace std;

//第二版一阶卡尔曼预测,角度版
class KF_two{
private:
    double pitch;                                       //测量pitch角度
    double yaw;                                       //测量yaw角度

public:

    Eigen::VectorXd x_;                         //状态向量[锁定目标绝对pitch,锁定目标绝对yaw,v_pitch,v_yaw]


    KF_two(Eigen::MatrixXd P_in , Eigen::MatrixXd Q_in,Eigen::MatrixXd H_in,Eigen::MatrixXd R_in);
    void Prediction(Eigen::MatrixXd _F);
    Eigen::VectorXd GetPrediction();
    Eigen::VectorXd GetPrediction(Eigen::MatrixXd _F);

        Eigen::MatrixXd F;                           //状态转移矩阵

        Eigen::MatrixXd P;                          //状态协方差矩阵
        Eigen::MatrixXd Q;                          //过程噪声

        Eigen::MatrixXd H;                          //测量矩阵

        Eigen::MatrixXd R;                          //测量噪声矩阵
        Eigen::MatrixXd K;                          //卡尔曼增益

        bool is_set_x = false;                     //判断是否赋初值

    KF_two();                                          //创建
    void set_x(Eigen::VectorXd x,Eigen::MatrixXd _F);                                   //赋初值
     void set_x(Eigen::VectorXd x);                                   //赋初值
    void update(Eigen::VectorXd z,Eigen::MatrixXd _F);             //更新
    Eigen::VectorXd get_x();                                  //返回状态向量
//    void initialize();                                    //初始化

};

#endif // FILTER_H
