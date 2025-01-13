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

    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr foxglove_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void subscription_callback_foxglove(const std_msgs::msg::Int32::SharedPtr foxglove_msg);
    // void subscription_callback_start(const std_msgs::msg::Int32::SharedPtr start_msg);
    void timer_callback(void);
    void LineFollowing(const ai_msgs::msg::Target &target);
    void ObstaclesAvoiding(const ai_msgs::msg::Target &ob_target);
    void park(void);
    void QR_follow(int x, int y);
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
    bool sub_target_ = false;
    std::shared_ptr<std::thread> msg_process_;

    float confidence_threshold_;
    int ob_bottom_th_in_I;
    int ob_bottom_th_in_III;
    int re_tim;
    float re_vel;
    float follow_linear_speed_I;
    float follow_angular_ratio_I;
    float follow_linear_speed_III;
    float follow_angular_ratio_III;
    int line_ob_x_th;
    float avoid_linear_speed_I;
    float avoid_angular_ratio_I;
    int ob_bottom_th_I;
    int tim_I;
    float avoid_linear_speed_III;
    float avoid_angular_ratio_III;
    int ob_bottom_th_III;
    int tim_III;
    int temp_lim;
    int modify_cnt_th;
    float modify_ratio;
    int P_cnt_th;
    int line_cnt_th;
    int P_bottom_th;
    float park_angular_ratio;
    float stop_linear_vel;
    float stop_linear_vel_close;
    int stop_cnt_th;
    float QR_follow_raio;
    float QR_follow_vel;

};


#endif  // RACING_CONTROL_H_
