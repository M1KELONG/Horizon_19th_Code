// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RACING_CONTROL_H_
#define RACING_CONTROL_H_

#include <vector>
#include <queue>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "origincar_msg/msg/sign.hpp"
#include "origincar_msg/msg/data.hpp"



// struct compare_point {
//   bool operator()(const geometry_msgs::msg::PointStamped::SharedPtr f1,
//                   const geometry_msgs::msg::PointStamped::SharedPtr f2) {
//     return ((f1->header.stamp.sec > f2->header.stamp.sec) ||
//             ((f1->header.stamp.sec == f2->header.stamp.sec) &&
//              (f1->header.stamp.nanosec > f2->header.stamp.nanosec)));
//   }
// };
// struct compare_target {
//   bool operator()(const ai_msgs::msg::PerceptionTargets::SharedPtr m1,
//                   const ai_msgs::msg::PerceptionTargets::SharedPtr m2) {
//     return ((m1->header.stamp.sec > m2->header.stamp.sec) ||
//             ((m1->header.stamp.sec == m2->header.stamp.sec) &&
//              (m1->header.stamp.nanosec > m2->header.stamp.nanosec)));
//   }
// };

class RacingControlNode : public rclcpp::Node
{
public:
  RacingControlNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RacingControlNode() override;

private:
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr foxglove_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr foxglove_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void subscription_callback_foxglove(const std_msgs::msg::Int32::SharedPtr foxglove_msg);
    void timer_callback(void);
    void LineFollowing(const ai_msgs::msg::Target &target);
    void ObstaclesAvoiding(const ai_msgs::msg::Target &ob_target, const ai_msgs::msg::Target &line_target);
    void park(void);
    void Qrtracking(void);
    void MessageProcess(void);

    int data_=0;
    std::string pub_control_topic_ = "cmd_vel";

    std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>>
    targets_queue_;
    // std::priority_queue<std_msgs::msg::Int32::SharedPtr,
    //                   std::vector<std_msgs::msg::Int32::SharedPtr>>
    // line_flag_queue_;

    std::mutex point_target_mutex_;

    bool process_stop_ = false;
    std::shared_ptr<std::thread> msg_process_;

    float follow_angular_ratio_ = -15.0;
    float follow_linear_speed_ ;
    float follow_linear_speed_III;
    int bottom_threshold_III ; 
    int ob_line_x ; 
    bool sub_target_ = false; 
    int bottom_threshold_ = 410;
    int bottom_threshold_ob = 320;
    int bottom_threshold_ob_III = 330;
    int END_CNT_threshold = 12;
    float confidence_threshold_ = 0.5;
    //  temp值限制
    float temp_lim = 20; 
    float ob_offset = 20;
    float ob_offset_III = 30;
    
    float re_vel = 0.2;
    int re_tim = 200; 
    float stop_linear_vel_close=0.05;
    float stop_linear_vel=0.3;
    float QR_speed=0.5;
    float qr_angular_ratio=-0.5;
    int stop_cnt_th=50;
    int line_cnt_th=10;
    int ob_cnt_th=10;
    float park_angular_ratio=2.0; 
    int P_bottom_th=400;


    // # =============
    // # ==任务一参数==
    // # =============
    // # 先左转：
    // # 第一段：
    // # 角度比例：
    float avoid_angular_ratio1_l_I_ = 20.0;
    // # 线速度：
    float avoid_linear_speed1_l_I_ = 0.3;
    // # 时间：
    int time1_l_I_ = 200;

    // # 第二段：
    // # 角度比例：
    float avoid_angular_ratio2_l_I_ = 20.0;
    // # 线速度：
    float avoid_linear_speed2_l_I_ = 0.3;
    // # 时间：
    int time2_l_I_ = 500;

    // # 第三段：
    // # 角度比例：
    float avoid_angular_ratio3_l_I_ = 20.0;
    // # 线速度：
    float avoid_linear_speed3_l_I_ = 0.3;
    // # 时间：
    int time3_l_I_ = 500;


    // # 先右转：
    // # 第一段：
    // # 角度比例：
    float avoid_angular_ratio1_r_I_ = 20.0;
    // # 线速度：
    float avoid_linear_speed1_r_I_ = 0.3;
    // # 时间：
    int time1_r_I_ = 200;

    // # 第二段：
    // # 角度比例：
    float avoid_angular_ratio2_r_I_ = 20.0;
    // # 线速度：
    float avoid_linear_speed2_r_I_ = 0.3;
    // # 时间：
    int time2_r_I_ = 500;

    // # 第三段：
    // # 角度比例：
    float avoid_angular_ratio3_r_I_ = 20.0;
    // # 线速度：
    float avoid_linear_speed3_r_I_ = 0.3;
    // # 时间：
    int time3_r_I_ = 500;



    // # =============
    // # ==任务三参数==
    // # =============
    // # 先左转：
    // # 第一段：
    // # 角度比例：
    float avoid_angular_ratio1_l_III_ = 20.0;
    // # 线速度：
    float avoid_linear_speed1_l_III_ = 0.3;
    // # 时间：
    int time1_l_III_ = 200;

    // # 第二段：
    // # 角度比例：
    float avoid_angular_ratio2_l_III_ = 20.0;
    // # 线速度：
    float avoid_linear_speed2_l_III_ = 0.3;
    // # 时间：
    int time2_l_III_ = 500;

    // # 第三段：
    // # 角度比例：
    float avoid_angular_ratio3_l_III_ = 20.0;
    // # 线速度：
    float avoid_linear_speed3_l_III_ = 0.3;
    // # 时间：
    int time3_l_III_ = 500;


    // # 先右转：
    // # 第一段：
    // # 角度比例：
    float avoid_angular_ratio1_r_III_ = 20.0;
    // # 线速度：
    float avoid_linear_speed1_r_III_ = 0.3;
    // # 时间：
    int time1_r_III_ = 200;

    // # 第二段：
    // # 角度比例：
    float avoid_angular_ratio2_r_III_ = 20.0;
    // # 线速度：
    float avoid_linear_speed2_r_III_ = 0.3;
    // # 时间：
    int time2_r_III_ = 500;

    // # 第三段：
    // # 角度比例：
    float avoid_angular_ratio3_r_III_ = 20.0;
    // # 线速度：
    float avoid_linear_speed3_r_III_ = 0.3;
    // # 时间：
    int time3_r_III_ = 500;

};


#endif  // RACING_CONTROL_H_