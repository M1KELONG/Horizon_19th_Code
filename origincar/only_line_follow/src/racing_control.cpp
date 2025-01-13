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

#include "racing_control/racing_control.h"
#include <unistd.h>
#include <math.h>

// bool start_flag = false;
bool avoid_flag = false;
bool ob2line_flag = false;
// int start_data = 0;
int foxglove_data = 0;
int END_CNT = 0;
int line_index_close;
int line_index_far;
// bool END_flag = false;
bool recognize_line = false;
bool recognize_P = false;
bool recognize_QR = false;
bool QRcode_flag = false;   // 满足置信度才真
bool QR_flag = false;       // 看到二维码就真
bool ob_flag = false;
bool P_flag = false;
int ob_center = 0;
int QR_x = 0;
int QR_y = 0;
int P_x = 0;
int P_y = 0;
int P_bottom = 0;
bool mission2_flag = false;
bool mission3_flag = false;
std::string qrcode_data;
static int cnt_ms2 = 0;
std::string state_flag;
auto P_vec=ai_msgs::msg::Target();
float follow_line_speed = 0.0;
int recover_cnt;
int recover_cnt_th;

RacingControlNode::RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
  if (!msg_process_)
  {
    msg_process_ = std::make_shared<std::thread>(
        std::bind(&RacingControlNode::MessageProcess, this));
  }
  this->declare_parameter<std::string>("pub_control_topic", pub_control_topic_);
  this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);

  this->declare_parameter<int>("temp_lim", temp_lim);
  this->get_parameter<int>("temp_lim", temp_lim);

  this->declare_parameter<int>("END_CNT_threshold", END_CNT_threshold);
  this->get_parameter<int>("END_CNT_threshold", END_CNT_threshold);
  
  this->declare_parameter<int>("P_bottom_th", P_bottom_th);
  this->get_parameter<int>("P_bottom_th", P_bottom_th);

  this->declare_parameter<float>("QR_confidence_th", QR_confidence_th);
  this->get_parameter<float>("QR_confidence_th", QR_confidence_th);

  this->declare_parameter<float>("QR_follow_vel", QR_follow_vel);
  this->get_parameter<float>("QR_follow_vel", QR_follow_vel);
  this->declare_parameter<float>("QR_follow_raio", QR_follow_raio);
  this->get_parameter<float>("QR_follow_raio", QR_follow_raio);
  
  this->declare_parameter<int>("ob_line_x_I", ob_line_x_I);
  this->get_parameter<int>("ob_line_x_I", ob_line_x_I);
  this->declare_parameter<int>("ob_line_x_III", ob_line_x_III);
  this->get_parameter<int>("ob_line_x_III", ob_line_x_III);
  this->declare_parameter<int>("QR_cnt_th", QR_cnt_th);
  this->get_parameter<int>("QR_cnt_th", QR_cnt_th);
  this->declare_parameter<int>("QR_x_th", QR_x_th);
  this->get_parameter<int>("QR_x_th", QR_x_th);
  this->declare_parameter<int>("ob_x_error_l_th", ob_x_error_l_th);
  this->get_parameter<int>("ob_x_error_l_th", ob_x_error_l_th);
  this->declare_parameter<int>("ob_x_error_r_th", ob_x_error_r_th);
  this->get_parameter<int>("ob_x_error_r_th", ob_x_error_r_th);
  this->declare_parameter<int>("ob_QR_th", ob_QR_th);
  this->get_parameter<int>("ob_QR_th", ob_QR_th);

  this->declare_parameter<float>("avoid_angular_ratio1_l_I_", avoid_angular_ratio1_l_I_);
  this->get_parameter<float>("avoid_angular_ratio1_l_I_", avoid_angular_ratio1_l_I_);
  this->declare_parameter<float>("avoid_angular_ratio2_l_I_", avoid_angular_ratio2_l_I_);
  this->get_parameter<float>("avoid_angular_ratio2_l_I_", avoid_angular_ratio2_l_I_);
  this->declare_parameter<float>("avoid_angular_ratio3_l_I_", avoid_angular_ratio3_l_I_);
  this->get_parameter<float>("avoid_angular_ratio3_l_I_", avoid_angular_ratio3_l_I_);

  this->declare_parameter<float>("avoid_angular_ratio1_l_III_", avoid_angular_ratio1_l_III_);
  this->get_parameter<float>("avoid_angular_ratio1_l_III_", avoid_angular_ratio1_l_III_);
  this->declare_parameter<float>("avoid_angular_ratio2_l_III_", avoid_angular_ratio2_l_III_);
  this->get_parameter<float>("avoid_angular_ratio2_l_III_", avoid_angular_ratio2_l_III_);
  this->declare_parameter<float>("avoid_angular_ratio3_l_III_", avoid_angular_ratio3_l_III_);
  this->get_parameter<float>("avoid_angular_ratio3_l_III_", avoid_angular_ratio3_l_III_);

  this->declare_parameter<float>("avoid_angular_ratio1_r_I_", avoid_angular_ratio1_r_I_);
  this->get_parameter<float>("avoid_angular_ratio1_r_I_", avoid_angular_ratio1_r_I_);
  this->declare_parameter<float>("avoid_angular_ratio2_r_I_", avoid_angular_ratio2_r_I_);
  this->get_parameter<float>("avoid_angular_ratio2_r_I_", avoid_angular_ratio2_r_I_);
  this->declare_parameter<float>("avoid_angular_ratio3_r_I_", avoid_angular_ratio3_r_I_);
  this->get_parameter<float>("avoid_angular_ratio3_r_I_", avoid_angular_ratio3_r_I_);

  this->declare_parameter<float>("avoid_angular_ratio1_r_III_", avoid_angular_ratio1_r_III_);
  this->get_parameter<float>("avoid_angular_ratio1_r_III_", avoid_angular_ratio1_r_III_);
  this->declare_parameter<float>("avoid_angular_ratio2_r_III_", avoid_angular_ratio2_r_III_);
  this->get_parameter<float>("avoid_angular_ratio2_r_III_", avoid_angular_ratio2_r_III_);
  this->declare_parameter<float>("avoid_angular_ratio3_r_III_", avoid_angular_ratio3_r_III_);
  this->get_parameter<float>("avoid_angular_ratio3_r_III_", avoid_angular_ratio3_r_III_);

  this->declare_parameter<float>("avoid_linear_speed1_l_I_", avoid_linear_speed1_l_I_);
  this->get_parameter<float>("avoid_linear_speed1_l_I_", avoid_linear_speed1_l_I_);
  this->declare_parameter<float>("avoid_linear_speed2_l_I_", avoid_linear_speed2_l_I_);
  this->get_parameter<float>("avoid_linear_speed2_l_I_", avoid_linear_speed2_l_I_);
  this->declare_parameter<float>("avoid_linear_speed3_l_I_", avoid_linear_speed3_l_I_);
  this->get_parameter<float>("avoid_linear_speed3_l_I_", avoid_linear_speed3_l_I_);

  this->declare_parameter<float>("avoid_linear_speed1_r_I_", avoid_linear_speed1_r_I_);
  this->get_parameter<float>("avoid_linear_speed1_r_I_", avoid_linear_speed1_r_I_);
  this->declare_parameter<float>("avoid_linear_speed2_r_I_", avoid_linear_speed2_r_I_);
  this->get_parameter<float>("avoid_linear_speed2_r_I_", avoid_linear_speed2_r_I_);
  this->declare_parameter<float>("avoid_linear_speed3_r_I_", avoid_linear_speed3_r_I_);
  this->get_parameter<float>("avoid_linear_speed3_r_I_", avoid_linear_speed3_r_I_);

  this->declare_parameter<float>("avoid_linear_speed1_l_III_", avoid_linear_speed1_l_III_);
  this->get_parameter<float>("avoid_linear_speed1_l_III_", avoid_linear_speed1_l_III_);
  this->declare_parameter<float>("avoid_linear_speed2_l_III_", avoid_linear_speed2_l_III_);
  this->get_parameter<float>("avoid_linear_speed2_l_III_", avoid_linear_speed2_l_III_);
  this->declare_parameter<float>("avoid_linear_speed3_l_III_", avoid_linear_speed3_l_III_);
  this->get_parameter<float>("avoid_linear_speed3_l_III_", avoid_linear_speed3_l_III_);

  this->declare_parameter<float>("avoid_linear_speed1_r_III_", avoid_linear_speed1_r_III_);
  this->get_parameter<float>("avoid_linear_speed1_r_III_", avoid_linear_speed1_r_III_);
  this->declare_parameter<float>("avoid_linear_speed2_r_III_", avoid_linear_speed2_r_III_);
  this->get_parameter<float>("avoid_linear_speed2_r_III_", avoid_linear_speed2_r_III_);
  this->declare_parameter<float>("avoid_linear_speed3_r_III_", avoid_linear_speed3_r_III_);
  this->get_parameter<float>("avoid_linear_speed3_r_III_", avoid_linear_speed3_r_III_);

  this->declare_parameter<int>("bottom_threshold", bottom_threshold_);
  this->get_parameter<int>("bottom_threshold", bottom_threshold_);

  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);

  this->declare_parameter<float>("follow_angular_ratio_I", follow_angular_ratio_I);
  this->get_parameter<float>("follow_angular_ratio_I", follow_angular_ratio_I);
  this->declare_parameter<float>("follow_angular_ratio_III", follow_angular_ratio_III);
  this->get_parameter<float>("follow_angular_ratio_III", follow_angular_ratio_III);

  this->declare_parameter<float>("confidence_threshold", confidence_threshold_);
  this->get_parameter<float>("confidence_threshold", confidence_threshold_);

  this->declare_parameter<int>("time1_l_I_", time1_l_I_);
  this->get_parameter<int>("time1_l_I_", time1_l_I_);
  this->declare_parameter<int>("time2_l_I_", time2_l_I_);
  this->get_parameter<int>("time2_l_I_", time2_l_I_);
  this->declare_parameter<int>("time3_l_I_", time3_l_I_);
  this->get_parameter<int>("time3_l_I_", time3_l_I_);

  this->declare_parameter<int>("time1_l_III_", time1_l_III_);
  this->get_parameter<int>("time1_l_III_", time1_l_III_);
  this->declare_parameter<int>("time2_l_III_", time2_l_III_);
  this->get_parameter<int>("time2_l_III_", time2_l_III_);
  this->declare_parameter<int>("time3_l_III_", time3_l_III_);
  this->get_parameter<int>("time3_l_III_", time3_l_III_);

  this->declare_parameter<int>("time1_r_I_", time1_r_I_);
  this->get_parameter<int>("time1_r_I_", time1_r_I_);
  this->declare_parameter<int>("time2_r_I_", time2_r_I_);
  this->get_parameter<int>("time2_r_I_", time2_r_I_);
  this->declare_parameter<int>("time3_r_I_", time3_r_I_);
  this->get_parameter<int>("time3_r_I_", time3_r_I_);

  this->declare_parameter<int>("time1_r_III_", time1_r_III_);
  this->get_parameter<int>("time1_r_III_", time1_r_III_);
  this->declare_parameter<int>("time2_r_III_", time2_r_III_);
  this->get_parameter<int>("time2_r_III_", time2_r_III_);
  this->declare_parameter<int>("time3_r_III_", time3_r_III_);
  this->get_parameter<int>("time3_r_III_", time3_r_III_);

  this->declare_parameter<float>("re_vel", re_vel);
  this->get_parameter<float>("re_vel", re_vel);
  this->declare_parameter<int>("re_tim", re_tim);
  this->get_parameter<int>("re_tim", re_tim);
  this->declare_parameter<int>("line_cnt_th", line_cnt_th);
  this->get_parameter<int>("line_cnt_th", line_cnt_th);
  this->declare_parameter<int>("P_cnt_th", P_cnt_th);
  this->get_parameter<int>("P_cnt_th", P_cnt_th);
  this->declare_parameter<int>("stop_cnt_th", stop_cnt_th);
  this->get_parameter<int>("stop_cnt_th", stop_cnt_th);
  this->declare_parameter<int>("recover_cnt", recover_cnt);
  this->get_parameter<int>("recover_cnt", recover_cnt);
  this->declare_parameter<float>("stop_linear_vel", stop_linear_vel);
  this->get_parameter<float>("stop_linear_vel", stop_linear_vel);
  this->declare_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
  this->get_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
  this->declare_parameter<float>("park_angular_ratio", park_angular_ratio);
  this->get_parameter<float>("park_angular_ratio", park_angular_ratio);

  this->declare_parameter<int>("bottom_threshold_III", bottom_threshold_III);
  this->get_parameter<int>("bottom_threshold_III", bottom_threshold_III);
  this->declare_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
  this->get_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
  this->declare_parameter<float>("P_confidence_th", P_confidence_th);
  this->get_parameter<float>("P_confidence_th", P_confidence_th);
  this->declare_parameter<float>("recognize_P_conf_th", recognize_P_conf_th);
  this->get_parameter<float>("recognize_P_conf_th", recognize_P_conf_th);

  qrcoder_subscriber_ =
      this->create_subscription<std_msgs::msg::String>(
          "foxglove_case",
          10,
          std::bind(&RacingControlNode::subscription_callback_qrcoder,
                    this,
                    std::placeholders::_1));

  // start_sub_ =
  //     this->create_subscription<std_msgs::msg::Int32>(
  //         "start_sign",
  //         10,
  //         std::bind(&RacingControlNode::subscription_callback_start,
  //                   this,
  //                   std::placeholders::_1));

  foxglove_subscriber_ =
      this->create_subscription<std_msgs::msg::Int32>(
          "sign4return",
          10,
          std::bind(&RacingControlNode::subscription_callback_foxglove,
                    this,
                    std::placeholders::_1));

  target_subscriber_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
          "racing_obstacle_detection",
          // "hobot_dnn_detection",
          10,
          std::bind(&RacingControlNode::subscription_callback_target,
                    this,
                    std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RacingControlNode::timer_callback, this));

  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, 5);

  foxglove_pub_ = 
      this->create_publisher<origincar_msg::msg::Sign>("sign_switch", 10);

  data_ = 0;
  recover_cnt_th = recover_cnt;
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "RacingControlNode initialized!");
}

RacingControlNode::~RacingControlNode()
{
  if (msg_process_ && msg_process_->joinable())
  {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    while (!targets_queue_.empty())
    {
      targets_queue_.pop();
    }
  }
}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg)
{
  {
    int ob_far = 640;
    sub_target_ = true;
    recognize_line = false;
    ob_flag = false;
    recognize_P = false;
    recognize_QR = false;
    QR_x = 0;
    QR_y = 0;
    P_x = 0;
    P_y = 0;
    P_bottom = 0;
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    targets_queue_.push(targets_msg);
    if (targets_queue_.size() > 1)
    {
      targets_queue_.pop();
    }
    
    /*在这里可以实现实时的障碍物检测*/
    if (targets_msg != nullptr)
    {
      for (const auto &target : targets_msg->targets)
      {
        if (target.type == "line")
        {
          recognize_line = true;
        }
        else if ((target.type == "clock") || (target.type == "anticlock"))
        {
          // recognize_QR = true;
          QR_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
          QR_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
          if (((QR_x - 320) < QR_x_th) && ((QR_x - 320) > -QR_x_th))
          {
            recognize_QR = true;
          }
          if(target.rois[0].confidence > QR_confidence_th)
          {
            QRcode_flag = true;
            if (target.type == "clock")
            {
              data_ = 3;
            }
            else
            {
              data_ = 4;
            }
          }
        }
        else if (target.type == "P")
        {
          if (target.rois[0].confidence > recognize_P_conf_th) 
          {
            recognize_P = true;
            P_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
            P_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
            P_bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
          }
        }
        else if (target.type == "obstacle")
        {
          ob_flag == true;
          int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
          // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of bottom is: %d", bottom);
          if (bottom < ob_far)
          {
            ob_far = bottom;
            ob_center = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
          }
          // ob_center = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
        }
      }
    }
  }
  return;
}

// void RacingControlNode::subscription_callback_start(const std_msgs::msg::Int32::SharedPtr start_msg)
// {
//   {
//     start_data = start_msg->data;
//     if (start_data == 8)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Strat!!");
//       start_flag = true;
//     }
//   }
//   return;
// }

void RacingControlNode::subscription_callback_qrcoder(const std_msgs::msg::String::SharedPtr qrcode_msg)
{
  {
    qrcode_data = qrcode_msg->data;
    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), qrcode_data);
    if ((qrcode_data == "ClockWise") || (qrcode_data == "AntiClockWise"))
    {
      QRcode_flag = true;
      if (qrcode_data == "ClockWise")
      {
        data_ = 3;
      }
      else
      {
        data_ = 4;
      }
    }
  }
  return;
}

void RacingControlNode::subscription_callback_foxglove(const std_msgs::msg::Int32::SharedPtr foxglove_msg)
{
  {
    foxglove_data = foxglove_msg->data;
    if (foxglove_data == 5)
    {
      mission2_flag = true;
    }
    if (foxglove_data == 6)
    {
      mission2_flag = false;
      mission3_flag = true;
      QRcode_flag = false;
    }
  }
  return;
}

void RacingControlNode::timer_callback()
    {
        // 创建消息类型为your_package_name::msg::Sign
        auto out_msg = origincar_msg::msg::Sign();
        // 将self.data赋值给消息的sign_data字段
        out_msg.sign_data = data_;
        // 发布消息
        foxglove_pub_->publish(out_msg);
    }

void RacingControlNode::MessageProcess()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  int QR_cnt = 0;
  int P_cnt = 0;
  while (process_stop_ == false)
  {
    int QR_center = 0;
    int bottom_temp = 0;
    int ob_x_error = 0;
    int ob_x_error_th;
    int bottom_temp_line_close = 0;
    int bottom_temp_line_far = 481;
    int cnt_target = 0;
    // int QR_x = 0;
    // int QR_y = 0;
    int target_index;
    line_index_close = 0;
    line_index_far = 0;
    int P_index=0;
    bool Obstacle_flag = false;
    bool Line_flag = false;
    P_flag = false;
    this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);
    this->get_parameter<int>("temp_lim", temp_lim);
    this->get_parameter<int>("END_CNT_threshold", END_CNT_threshold);
    this->get_parameter<int>("P_bottom_th", P_bottom_th);
    this->get_parameter<float>("QR_confidence_th", QR_confidence_th);
    this->get_parameter<float>("QR_follow_vel", QR_follow_vel);
    this->get_parameter<float>("QR_follow_raio", QR_follow_raio);
    this->get_parameter<int>("ob_line_x_I", ob_line_x_I);
    this->get_parameter<int>("ob_line_x_III", ob_line_x_III);
    this->get_parameter<int>("ob_QR_th", ob_QR_th);
    this->get_parameter<int>("QR_cnt_th", QR_cnt_th);
    this->get_parameter<int>("QR_x_th", QR_x_th);
    this->get_parameter<int>("ob_x_error_l_th", ob_x_error_l_th);
    this->get_parameter<int>("ob_x_error_r_th", ob_x_error_r_th);
    this->get_parameter<float>("avoid_angular_ratio1_l_I_", avoid_angular_ratio1_l_I_);
    this->get_parameter<float>("avoid_angular_ratio2_l_I_", avoid_angular_ratio2_l_I_);
    this->get_parameter<float>("avoid_angular_ratio3_l_I_", avoid_angular_ratio3_l_I_);
    this->get_parameter<float>("avoid_angular_ratio1_l_III_", avoid_angular_ratio1_l_III_);
    this->get_parameter<float>("avoid_angular_ratio2_l_III_", avoid_angular_ratio2_l_III_);
    this->get_parameter<float>("avoid_angular_ratio3_l_III_", avoid_angular_ratio3_l_III_);
    this->get_parameter<float>("avoid_angular_ratio1_r_I_", avoid_angular_ratio1_r_I_);
    this->get_parameter<float>("avoid_angular_ratio2_r_I_", avoid_angular_ratio2_r_I_);
    this->get_parameter<float>("avoid_angular_ratio3_r_I_", avoid_angular_ratio3_r_I_);
    this->get_parameter<float>("avoid_angular_ratio1_r_III_", avoid_angular_ratio1_r_III_);
    this->get_parameter<float>("avoid_angular_ratio2_r_III_", avoid_angular_ratio2_r_III_);
    this->get_parameter<float>("avoid_angular_ratio3_r_III_", avoid_angular_ratio3_r_III_);
    this->get_parameter<float>("avoid_linear_speed1_l_I_", avoid_linear_speed1_l_I_);
    this->get_parameter<float>("avoid_linear_speed2_l_I_", avoid_linear_speed2_l_I_);
    this->get_parameter<float>("avoid_linear_speed3_l_I_", avoid_linear_speed3_l_I_);
    this->get_parameter<float>("avoid_linear_speed1_r_I_", avoid_linear_speed1_r_I_);
    this->get_parameter<float>("avoid_linear_speed2_r_I_", avoid_linear_speed2_r_I_);
    this->get_parameter<float>("avoid_linear_speed3_r_I_", avoid_linear_speed3_r_I_);
    this->get_parameter<float>("avoid_linear_speed1_l_III_", avoid_linear_speed1_l_III_);
    this->get_parameter<float>("avoid_linear_speed2_l_III_", avoid_linear_speed2_l_III_);
    this->get_parameter<float>("avoid_linear_speed3_l_III_", avoid_linear_speed3_l_III_);
    this->get_parameter<float>("avoid_linear_speed1_r_III_", avoid_linear_speed1_r_III_);
    this->get_parameter<float>("avoid_linear_speed2_r_III_", avoid_linear_speed2_r_III_);
    this->get_parameter<float>("avoid_linear_speed3_r_III_", avoid_linear_speed3_r_III_);
    this->get_parameter<int>("bottom_threshold", bottom_threshold_);
    this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
    this->get_parameter<float>("follow_angular_ratio_I", follow_angular_ratio_I);
    this->get_parameter<float>("follow_angular_ratio_III", follow_angular_ratio_III);
    this->get_parameter<int>("bottom_threshold_III", bottom_threshold_III);
    this->get_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
    this->get_parameter<float>("confidence_threshold", confidence_threshold_);
    this->get_parameter<int>("time1_l_I_", time1_l_I_);
    this->get_parameter<int>("time2_l_I_", time2_l_I_);
    this->get_parameter<int>("time3_l_I_", time3_l_I_);
    this->get_parameter<int>("time1_l_III_", time1_l_III_);
    this->get_parameter<int>("time2_l_III_", time2_l_III_);
    this->get_parameter<int>("time3_l_III_", time3_l_III_);
    this->get_parameter<int>("time1_r_I_", time1_r_I_);
    this->get_parameter<int>("time2_r_I_", time2_r_I_);
    this->get_parameter<int>("time3_r_I_", time3_r_I_);
    this->get_parameter<int>("time1_r_III_", time1_r_III_);
    this->get_parameter<int>("time2_r_III_", time2_r_III_);
    this->get_parameter<int>("time3_r_III_", time3_r_III_);
    this->get_parameter<float>("re_vel", re_vel);
    this->get_parameter<int>("re_tim", re_tim);
    this->get_parameter<int>("line_cnt_th", line_cnt_th);
    this->get_parameter<int>("P_cnt_th", P_cnt_th);
    this->get_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
    this->get_parameter<float>("stop_linear_vel", stop_linear_vel);
    this->get_parameter<int>("stop_cnt_th", stop_cnt_th);
    this->get_parameter<int>("recover_cnt", recover_cnt);
    this->get_parameter<float>("park_angular_ratio", park_angular_ratio);
    this->get_parameter<float>("P_confidence_th", P_confidence_th);
    this->get_parameter<float>("recognize_P_conf_th", recognize_P_conf_th);

    // 正在进行任务二
    if (mission2_flag == true)
    {
      if (cnt_ms2 < 1)
      {
        RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Mission2");
        cnt_ms2++;
      }
    }
    // 没有进行任务二
    else
    {
      if (QRcode_flag == true)
      {
        if (state_flag != "QRcode_Stop")
        {
          RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "QRcode_Stop!");
          twist_msg.linear.x = -re_vel;
          twist_msg.angular.z = 0.0;
          publisher_->publish(twist_msg);
          usleep(re_tim * 1000);
          state_flag = "QRcode_Stop";
        }

        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
      }
      else
      {
        std::unique_lock<std::mutex> lock(point_target_mutex_);
        if (!targets_queue_.empty() && sub_target_ == true)
        {
          // auto point_msg = point_queue_.top();
          auto targets_msg = targets_queue_.top();
          // point_queue_.pop();
          targets_queue_.pop();
          lock.unlock();
          for (const auto &target : targets_msg->targets)
          {
            if (target.type == "obstacle")
            {
              Obstacle_flag = true;
              // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected obstacle!");
              int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
              // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of bottom is: %d", bottom);
              if (bottom > bottom_temp)
              {
                bottom_temp = bottom;
                target_index = cnt_target;
                ob_x_error = (target.rois[0].rect.x_offset + target.rois[0].rect.width) - 320;
                // 障碍物在视野左边
                if (ob_x_error < 0)
                {
                  ob_x_error = -ob_x_error;
                  ob_x_error_th = ob_x_error_l_th;
                }
                // 障碍物在视野右边
                else
                {
                  ob_x_error_th = ob_x_error_r_th;
                }
              }
            }
            else if (target.type == "line")
            {
              // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected line!");
              Line_flag = true;
              int line_bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
              if (line_bottom > bottom_temp_line_close)
              {
                bottom_temp_line_close = line_bottom;
                line_index_close = cnt_target;
              }
              if (line_bottom < bottom_temp_line_far)
              {
                bottom_temp_line_far = line_bottom;
                line_index_far = cnt_target;
              }
              // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected line!");
              // float line_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
              // float line_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
            }
            else if (target.type == "clock")
            {
              QR_flag = true;
              QR_center = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
              // QR_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
              if(target.rois[0].confidence > QR_confidence_th)
              {
                data_ = 3;
                QRcode_flag = true;
              }
              
              // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected clock!");
            }
            else if (target.type == "anticlock")
            {
              QR_flag = true;
              QR_center = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
              // QR_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
              if(target.rois[0].confidence > QR_confidence_th)
              {
                data_ = 4;
                QRcode_flag = true;
              }
              // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected anticlock!");
            }
            else if (target.type == "P")
            {
              // P_flag = true;
              if (target.rois[0].confidence > P_confidence_th)
              {
                P_flag = true;
              }
            }
            cnt_target++;
          }
          /*任务1*/
          if (mission3_flag == false)
          {
            LineFollowing(targets_msg->targets[line_index_far]);
            // // 没看到障碍物
            // if (Obstacle_flag == false)
            // {
            //   if ((QR_flag == true) && ((QR_center-320)<QR_x_th) && ((QR_center-320)>-QR_x_th))
            //   {
            //     QR_cnt += 1;
            //   }
            //   if (QR_cnt > QR_cnt_th)
            //   {
            //     QR_follow();
            //   }
            //   else if (Line_flag == true)
            //   {
            //     QR_cnt = 0;
            //     LineFollowing(targets_msg->targets[line_index_far]);
            //     if (state_flag != "Follow line without ob")
            //     {
            //       RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line without obstacle!");
            //       state_flag = "Follow line without ob";
            //     }
            //   }
            // }
            // // 看到障碍物
            // else
            // {
            //   // 未到达阈值
            //   if (bottom_temp < bottom_threshold_)
            //   {
            //     // if (bottom_temp < bottom_threshold_ && recognize_line == true){
            //     if (Line_flag == true)
            //     {
            //       LineFollowing(targets_msg->targets[line_index_far]);
            //       if (state_flag != "Follow line with far ob")
            //       {
            //         RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line with far obstacle!");
            //         state_flag = "Follow line with far ob";
            //       }
            //     }
            //   }
            //   // 达到阈值
            //   else
            //   {
            //     // 障碍物不在边边
            //     if (ob_x_error < ob_x_error_th)
            //     {
            //       const auto &target1 = targets_msg->targets[target_index];
            //       if (target1.rois[0].confidence > confidence_threshold_)
            //       {
            //         ObstaclesAvoiding(target1,targets_msg->targets[line_index_far]);
            //       }
            //     }
            //     // 障碍物在边边
            //     else if (Line_flag == true)
            //     {
            //       LineFollowing(targets_msg->targets[line_index_far]);
            //     }
            //   }
            // }
          }
          /*任务3*/
          else if (mission3_flag == true)
          {
            LineFollowing(targets_msg->targets[line_index_far]);
            if (P_flag == true)
            {
              P_cnt += 1;
            }
            if (P_cnt > P_cnt_th)
            {
              park();
            }
            // if (Obstacle_flag == false)
            // {
            //   if (P_flag == true)
            //   {
            //     RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "P stop!");
            //     park();
            //   }
            //   if (Line_flag == true)
            //   {
            //     LineFollowing(targets_msg->targets[line_index_far]);
            //     if (state_flag != "Follow line without ob")
            //     {
            //       RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line without obstacle!");
            //       state_flag = "Follow line without ob";
            //     }
            //   }
            // }
            // else
            // {
            //   if (bottom_temp < bottom_threshold_III)
            //   {
            //     // if (bottom_temp < bottom_threshold_ && recognize_line == true){
            //     if (Line_flag == true)
            //     {
            //       LineFollowing(targets_msg->targets[line_index_far]);
            //       if (state_flag != "Follow line with far ob")
            //       {
            //         RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line with far obstacle!");
            //         state_flag = "Follow line with far ob";
            //       }
            //     }
            //   }
            //   else
            //   // else if (ob_x_error < ob_x_error_th)
            //   {
            //     const auto &target1 = targets_msg->targets[target_index];
            //     if (target1.rois[0].confidence > confidence_threshold_)
            //     {
            //       ObstaclesAvoiding(target1,targets_msg->targets[line_index_far]);
            //     }
            //   }
            // }
          }
          lock.lock();
        }
      }
    }
    // }
  }
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &target)
{
  float line_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float line_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
  float temp = line_x - 320.0;
  float ratio;
  if ((-20 < line_x) && (line_x < 0))
  {
    temp = -20;
  }
  else if ((line_x > 0) && (line_x < 20))
  {
    temp = 20;
  }
  auto twist_msg = geometry_msgs::msg::Twist();
  if (mission3_flag == false)
  {
    ratio = follow_angular_ratio_I;
    if (ob2line_flag == true)
    {
      if(recover_cnt != 0)
      {
        follow_line_speed = (follow_linear_speed_ - follow_line_speed)/ recover_cnt + follow_line_speed;
        recover_cnt--;
      }
      else
      {
        follow_line_speed = follow_linear_speed_;
        recover_cnt = recover_cnt_th;
        ob2line_flag = false;
      }
    }
    else
    {
      follow_line_speed = follow_linear_speed_;
      recover_cnt = recover_cnt_th;
    }
  }
  else
  {
    ratio = follow_angular_ratio_III;
    if (ob2line_flag == true)
    {
      if(recover_cnt != 0)
      {
        follow_line_speed = (follow_linear_speed_III - follow_line_speed)/ recover_cnt + follow_line_speed;
        recover_cnt--;
      }
      else
      {
        follow_line_speed = follow_linear_speed_III;
        recover_cnt = recover_cnt_th;
        ob2line_flag = false;
      }
    }
    else
    {
      follow_line_speed = follow_linear_speed_III;
      recover_cnt = recover_cnt_th;
    }
  }
  
  float angular_z = ratio * temp / 150.0 * line_y / 224.0;
  twist_msg.linear.x = follow_line_speed;
  // angular_z = value_limit(angular_z,32.5);
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
  avoid_flag = false;
}

void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &ob_target, const ai_msgs::msg::Target &line_target)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  int line_far_x = line_target.rois[0].rect.x_offset + line_target.rois[0].rect.width / 2;
  float angular_z1;
  float angular_z2;
  float angular_z3;
  float avoid_linear_speed1_;
  float avoid_linear_speed2_;
  float avoid_linear_speed3_;
  int tim1_;
  int tim2_;
  // int tim1_, tim2_, tim3_;
  // avoid_flag表示是否进行过避障
  if (avoid_flag == false)
  {
    if (state_flag != "Avoiding ob")
    {
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now avoiding obstacle");
      state_flag = "Avoiding ob";
    }

    int ob_x = ob_target.rois[0].rect.x_offset + ob_target.rois[0].rect.width / 2;
    float temp;
    int ob_line_th;

    // 既有障碍物，也有线，进行相对位置判断
    if (recognize_line)
    {
      float x_error = line_far_x - ob_x;
      if (mission3_flag == false)
      {
        ob_line_th = ob_line_x_I;
      }
      else
      {
        ob_line_th = ob_line_x_III;
      }
      // 认为障碍物在线上
      if ((x_error > -ob_line_th) && (x_error < ob_line_th))
      {
        temp = temp_lim;   // 强制左转
      }
      else
      {
        temp = 1.0 * (-x_error);
      }
    }
    // 只有障碍物
    else
    {
      temp = ob_x - 320.0;
    }

    if ((-temp_lim < temp) && (temp < 0))
    {
      temp = -temp_lim;
    }
    else if ((temp >= 0) && (temp < temp_lim))
    {
      temp = temp_lim;
    }

    // 判断任务一和三
    if (mission3_flag == false) // 任务一
    {
      // if (temp > - (temp_lim+5)) // 左转
      if (temp > 0) // 左转
      {
        // if(temp < 0)
        // {
        //   temp = -temp;
        // }
        tim1_ = time1_l_I_;
        tim2_ = time2_l_I_;
        // tim3_ = time3_l_;
        angular_z1 = avoid_angular_ratio1_l_I_ * 700 / temp;
        angular_z2 = avoid_angular_ratio2_l_I_ * 700 / temp;
        angular_z3 = avoid_angular_ratio3_l_I_;

        avoid_linear_speed1_ = avoid_linear_speed1_l_I_;
        avoid_linear_speed2_ = avoid_linear_speed2_l_I_;
        avoid_linear_speed3_ = avoid_linear_speed3_l_I_;
      }
      else // 右转
      {
        tim1_ = time1_r_I_;
        tim2_ = time2_r_I_;
        // tim3_ = time3_r_;
        angular_z1 = avoid_angular_ratio1_r_I_ * 700 / temp;
        angular_z2 = avoid_angular_ratio2_r_I_ * 700 / temp;
        angular_z3 = avoid_angular_ratio3_r_I_;

        avoid_linear_speed1_ = avoid_linear_speed1_r_I_;
        avoid_linear_speed2_ = avoid_linear_speed2_r_I_;
        avoid_linear_speed3_ = avoid_linear_speed3_r_I_;
      }

      // 第一段
      twist_msg.linear.x = avoid_linear_speed1_;
      // angular_z1 = value_limit(angular_z1,32.5);
      twist_msg.angular.z = angular_z1;
      publisher_->publish(twist_msg);
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to avoid obstacle1!");
      // usleep(tim1_ * 1000); // 休眠1000ms
      //
      // tim1_ = 800;
      usleep(tim1_ * 1000);

      // 3R-2
      twist_msg.linear.x = avoid_linear_speed2_;
      // angular_z2 = value_limit(angular_z2,32.5);
      twist_msg.angular.z = angular_z2;
      publisher_->publish(twist_msg);
      // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to return!");
      if (state_flag != "Sleep to return!")
      {
        RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to return!");
        state_flag = "Sleep to return!";
      }
      usleep(tim2_ * 1000); // 休眠1000ms
    }
    else // 任务三
    {
      if (temp > 0) // 左转
      {
        tim1_ = time1_l_III_;
        tim2_ = time2_l_III_;
        // tim3_ = time3_l_;
        angular_z1 = avoid_angular_ratio1_l_III_ * 700 / temp;
        angular_z2 = avoid_angular_ratio2_l_III_ * 700 / temp;
        angular_z3 = avoid_angular_ratio3_l_III_;

        avoid_linear_speed1_ = avoid_linear_speed1_l_III_;
        avoid_linear_speed2_ = avoid_linear_speed2_l_III_;
        avoid_linear_speed3_ = avoid_linear_speed3_l_III_;
      }
      else // 右转
      {
        tim1_ = time1_r_III_;
        tim2_ = time2_r_III_;
        // tim3_ = time3_r_;
        angular_z1 = avoid_angular_ratio1_r_III_ * 700 / temp;
        angular_z2 = avoid_angular_ratio2_r_III_ * 700 / temp;
        angular_z3 = avoid_angular_ratio3_r_III_;

        avoid_linear_speed1_ = avoid_linear_speed1_r_III_;
        avoid_linear_speed2_ = avoid_linear_speed2_r_III_;
        avoid_linear_speed3_ = avoid_linear_speed3_r_III_;
      }

      // 第一段
      twist_msg.linear.x = avoid_linear_speed1_;
      angular_z1 = value_limit(angular_z1,32.5);
      twist_msg.angular.z = angular_z1;
      publisher_->publish(twist_msg);
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to avoid obstacle1!");
      // usleep(tim1_ * 1000); // 休眠1000ms
      //
      // tim1_ = 800;
      usleep(tim1_ * 1000);

      // 3R-2
      twist_msg.linear.x = avoid_linear_speed2_;
      angular_z2 = value_limit(angular_z2,32.5);
      twist_msg.angular.z = angular_z2;
      publisher_->publish(twist_msg);
      // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to return!");
      if (state_flag != "Sleep to return!")
      {
        RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to return!");
        state_flag = "Sleep to return!";
      }
      usleep(tim2_ * 1000); // 休眠1000ms
    }

    avoid_flag = true;
    ob2line_flag = true;
    follow_line_speed = avoid_linear_speed3_;
  }

  int line_cnt = 0;
  int QR_cnt = 0;
  int P_cnt = 0;
  // 第三段
  // while (recognize_line != true) // 如果巡线没有传递线的消息，进行姿态调整
  while (true) // 如果巡线没有传递线的消息，进行姿态调整
  {
    if (recognize_line == true)
    {
      line_cnt += 1;
    }
    if (line_cnt > line_cnt_th)
    {
      break;
    }
    int QR_ob_error = QR_x - ob_center;
    if (QR_ob_error < 0)
    {
      QR_ob_error = -QR_ob_error;
    }
    // 根据是否看到线回巡线
    twist_msg.linear.x = avoid_linear_speed3_;
    // if (mission3_flag == true)
    // {
    //   angular_z3 = value_limit(angular_z3,32.5);
    // }
    twist_msg.angular.z = -angular_z3;
    // if (twist_msg.angular.z > 32.5)
    // {
    //   twist_msg.angular.z = 32.5;
    // }
    // else if (twist_msg.angular.z < -32.5)
    // {
    //   twist_msg.angular.z = -32.5;
    // }
    publisher_->publish(twist_msg);
    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "return!");
    if (mission3_flag == false)
    {
      // if (recognize_QR == true)
      if ((recognize_QR == true) && (QR_ob_error > ob_QR_th))
      // if ((recognize_QR == true) && (ob_flag == false))
      {
        QR_cnt += 1;
      }
      if (QR_cnt > QR_cnt_th)
      {
        RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "QR found!");
        QR_follow();
        break;
      }
      if (QRcode_flag == true)
      {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
        break;
      }
    }
    else
    {
      if (recognize_P == true)
      {
        P_cnt += 1;
      }
      if (P_cnt > P_cnt_th)
      {
        RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "P break!");
        park();
        break;
      }
    }
  }
  // return;
}

void RacingControlNode::park()
{
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Parking!");
  auto twist_msg = geometry_msgs::msg::Twist();
  int end_cnt = 0;
  bool flag = false;
  while(1)
  {
    // 看得到P
    if(recognize_P)
    {
      end_cnt = 0;

      // 判断是否已经靠近过P
      if (P_bottom >= P_bottom_th)
        flag=true;

      float temp = P_x - 320.0;

      if(temp>-20 && temp<=0)
        temp = -20;
      else if(temp>0 && temp<20) 
        temp = 20;

      float angular_z = park_angular_ratio * temp / 150.0 * P_y / 480.0;
      
      // 已经靠近过P，且还能看到P，应该以小速度往前走
      if (flag)
      {
        twist_msg.linear.x = stop_linear_vel;
        // angular_z = value_limit(angular_z,32.5);
        twist_msg.angular.z = -angular_z;
      }
      // 未靠近过P，且看到P，应该以大速度靠近
      else
      {
        twist_msg.linear.x = stop_linear_vel_close;
        // angular_z = value_limit(angular_z,32.5);
        twist_msg.angular.z = -angular_z;
      }
      publisher_->publish(twist_msg);
    }
    // 看不见P
    else 
    {
      // 认为已经踩在P上了
      if (flag)
      {
        end_cnt += 1;
      }
    }
    if (end_cnt >= stop_cnt_th)
    {
      break;
    }
  }
  while(1)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    publisher_->publish(twist_msg);
  }
}
    
void RacingControlNode::QR_follow()
{
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Following QRcode!");
  auto twist_msg = geometry_msgs::msg::Twist();
  float temp;
  float angular_z;
  while (1)
  {
    temp = QR_x - 320.0;

    if(temp>-20 && temp<=0)
      temp = -20;
    else if(temp>0 && temp<20) 
      temp = 20;

    angular_z = QR_follow_raio * temp / 150.0 * QR_y / 480.0;

    twist_msg.linear.x = QR_follow_vel;
    // angular_z = value_limit(angular_z,32.5);
    twist_msg.angular.z = -angular_z;
    publisher_->publish(twist_msg);

    if (QRcode_flag == true)
    {
      break;
    }
  }

  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  publisher_->publish(twist_msg);
}

float RacingControlNode::value_limit(float origin, float limit)
{
  float result;
  if (origin > limit)
  {
    result = limit;
  }
  else if (origin < (-limit))
  {
    result = -limit;
  }
  else
  {
    result = origin;
  }
  return result;
}

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
