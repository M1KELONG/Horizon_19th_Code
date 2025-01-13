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

bool avoid_flag = false;
bool ob2line_flag = false;
int foxglove_data = 0;
int line_cnt = 0;
int ob_cnt = 0;
// bool END_flag = false;
bool recognize_line = false;
bool recognize_P = false;
bool recognize_ob = false;
bool recognize_qr = false;
bool QRcode_flag = false;
bool track_qr_flag = false;
bool P_flag = false;
bool mission2_flag = false;
bool mission3_flag = false;
std::string qrcode_data;
static int cnt_ms2 = 0;
std::string state_flag;
auto P_vec=ai_msgs::msg::Target();
auto qr_vec=ai_msgs::msg::Target();
auto line_far=ai_msgs::msg::Target();
bool line_far_empty = true;
int recover_cnt;
int recover_cnt_th;
float follow_line_speed = 0.0;



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

  this->declare_parameter<float>("temp_lim", temp_lim);
  this->get_parameter<float>("temp_lim", temp_lim);

  this->declare_parameter<int>("END_CNT_threshold", END_CNT_threshold);
  this->get_parameter<int>("END_CNT_threshold", END_CNT_threshold);
  
  this->declare_parameter<int>("P_bottom_th", P_bottom_th);
  this->get_parameter<int>("P_bottom_th", P_bottom_th);

  this->declare_parameter<int>("ob_line_x", ob_line_x);
  this->get_parameter<int>("ob_line_x", ob_line_x);

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
  
  this->declare_parameter<int>("bottom_threshold_ob", bottom_threshold_ob);
  this->get_parameter<int>("bottom_threshold_ob", bottom_threshold_ob);
  this->declare_parameter<int>("bottom_threshold_ob_III", bottom_threshold_ob_III);
  this->get_parameter<int>("bottom_threshold_ob_III", bottom_threshold_ob_III);

  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);

  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);

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
  this->declare_parameter<int>("stop_cnt_th", stop_cnt_th);
  this->get_parameter<int>("stop_cnt_th", stop_cnt_th);
  this->declare_parameter<int>("line_cnt_th", line_cnt_th);
  this->get_parameter<int>("line_cnt_th", line_cnt_th);
  this->declare_parameter<int>("ob_cnt_th", ob_cnt_th);
  this->get_parameter<int>("ob_cnt_th", ob_cnt_th);
  this->declare_parameter<float>("ob_offset", ob_offset);
  this->get_parameter<float>("ob_offset", ob_offset);
  this->declare_parameter<float>("ob_offset_III", ob_offset_III);
  this->get_parameter<float>("ob_offset_III", ob_offset_III);
  this->declare_parameter<int>("recover_cnt", recover_cnt);
  this->get_parameter<int>("recover_cnt", recover_cnt);
  this->declare_parameter<float>("stop_linear_vel", stop_linear_vel);
  this->get_parameter<float>("stop_linear_vel", stop_linear_vel);
  this->declare_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
  this->get_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
  this->declare_parameter<float>("QR_speed", QR_speed);
  this->get_parameter<float>("QR_speed", QR_speed);
  this->declare_parameter<float>("qr_angular_ratio", qr_angular_ratio);
  this->get_parameter<float>("qr_angular_ratio", qr_angular_ratio);
  this->declare_parameter<float>("park_angular_ratio", park_angular_ratio);
  this->get_parameter<float>("park_angular_ratio", park_angular_ratio);

  this->declare_parameter<int>("bottom_threshold_III", bottom_threshold_III);
  this->get_parameter<int>("bottom_threshold_III", bottom_threshold_III);
  this->declare_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
  this->get_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);

  // point_subscriber_ =
  //     this->create_subscription<ai_msgs::msg::PerceptionTargets>(
  //         "racing_track_center_detection",
  //         10,
  //         std::bind(&RacingControlNode::subscription_callback_point,
  //                   this,
  //                   std::placeholders::_1));

  // qrcoder_subscriber_ =
      // this->create_subscription<std_msgs::msg::String>(
      //     "foxglove_case",
      //     10,
      //     std::bind(&RacingControlNode::subscription_callback_qrcoder,
      //               this,
      //               std::placeholders::_1));

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

  // line_flag_subscriber_ =
  //     this->create_subscription<std_msgs::msg::Int32>(
  //         "line_flag",
  //         10,
  //         std::bind(&RacingControlNode::subscription_callback_line_flag,
  //                   this,
  //                   std::placeholders::_1));

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

// void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg)
// {
//   {
//     std::unique_lock<std::mutex> lock(point_target_mutex_);
//     point_queue_.push(point_msg);
//     if (point_queue_.size() > 1)
//     {
//       point_queue_.pop();
//     }
//   }
//   return;
// }

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg)
{
  {
    sub_target_ = true;
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    targets_queue_.push(targets_msg);
    if (targets_queue_.size() > 1)
    {
      targets_queue_.pop();
    }
    int rec_bottom_line = 999;
    int rec_bottom_temp=0;
    bool found_P =false;
    bool found_line =false;
    bool found_qr =false;
    line_far_empty = true;
    static bool found_ob;
    if(recognize_ob == true)
    {
      ob_cnt++;
      if (ob_cnt > ob_cnt_th)
      {
        found_ob = false;
        ob_cnt = 0;
      }
    }
    else
    {
      found_ob = false;
      ob_cnt = 0;
    }
    /*在这里可以实现实时的障碍物检测*/
    if (targets_msg != nullptr)
    {
      for (const auto &target : targets_msg->targets)
      {
        if (target.type == "line")
        {
          line_cnt++;
          if (line_cnt > line_cnt_th)
          {
            found_line = true;
            line_cnt = 0;
          }
          int bottom_line = target.rois[0].rect.y_offset + target.rois[0].rect.height;
          if (bottom_line < rec_bottom_line)
          {
            rec_bottom_line = bottom_line;
            line_far = target;
            line_far_empty = false;
          }
        }
        if (target.type == "P")
        {
          found_P = true;
          P_vec = target;
        }
        if (target.type == "obstacle")
        {
          int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
          if (bottom > rec_bottom_temp)
          {
            rec_bottom_temp = bottom;
            // ob_vec = target;
          }
        }
        if ((target.type == "clock") || (target.type == "clock_r"))
        {
          found_qr = true;
          qr_vec = target;  
        }
        if ((target.type == "anticlock") || (target.type == "anticlock_r"))
        {
          found_qr = true;
          qr_vec = target;  
        }
      }
      // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of bottom is: %d", rec_bottom_temp);
      if (mission3_flag == false)
      {
        if (rec_bottom_temp > bottom_threshold_ob)
          found_ob = true;
      }
      else 
      {
        if (rec_bottom_temp > bottom_threshold_ob_III)
          found_ob = true;
      }
      
      recognize_ob = found_ob;
      recognize_line = found_line;
      recognize_P = found_P;
      recognize_qr = found_qr;
      // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of ob is: %d", recognize_ob);
      
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
    auto out_msg = origincar_msg::msg::Sign();
    // 将self.data赋值给消息的sign_data字段
    out_msg.sign_data = data_;
    // 发布消息
    foxglove_pub_->publish(out_msg);
}

void RacingControlNode::MessageProcess()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  while (process_stop_ == false)
  {
    int bottom_temp = 0;
    int bottom_temp_line_close = 0;
    int bottom_temp_line_far = 999;
    int cnt_target = 0;
    int target_index;
    int line_index_close;
    int line_index_far;
    bool Obstacle_flag = false;
    bool Line_flag = false;
    this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);
    this->get_parameter<float>("temp_lim", temp_lim);
    this->get_parameter<int>("END_CNT_threshold", END_CNT_threshold);
    this->get_parameter<int>("P_bottom_th", P_bottom_th);
    this->get_parameter<int>("ob_line_x", ob_line_x);
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
    this->get_parameter<int>("bottom_threshold_ob", bottom_threshold_ob);
    this->get_parameter<int>("bottom_threshold_ob_III", bottom_threshold_ob_III);
    this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
    this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
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
    this->get_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
    this->get_parameter<float>("stop_linear_vel", stop_linear_vel);
    this->get_parameter<float>("QR_speed", QR_speed);
    this->get_parameter<float>("qr_angular_ratio", qr_angular_ratio);
    this->get_parameter<int>("stop_cnt_th", stop_cnt_th);
    this->get_parameter<int>("line_cnt_th", line_cnt_th);
    this->get_parameter<int>("ob_cnt_th", ob_cnt_th);
    this->get_parameter<float>("ob_offset", ob_offset);
    this->get_parameter<float>("ob_offset_III", ob_offset_III);
    this->get_parameter<int>("recover_cnt", recover_cnt);
    this->get_parameter<float>("park_angular_ratio", park_angular_ratio);


    if (mission2_flag == true)
    {
      if (cnt_ms2 < 1)
      {
        RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Mission2");
        cnt_ms2++;
      }
    }
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
              // float line_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
              // float line_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
            }
            else if ((target.type == "clock") || (target.type == "clock_r"))
            {
              if(target.rois[0].confidence > 0.6)
              {
                if(target.rois[0].rect.x_offset + target.rois[0].rect.width > 100)
                  track_qr_flag = true;
              }
              if(target.rois[0].confidence > 0.8)
              {
                data_ = 3;
                QRcode_flag = true;
              }
              
            }
            else if ((target.type == "anticlock") || (target.type == "anticlock_r"))
            {
              if(target.rois[0].confidence > 0.6)
              {
                if(target.rois[0].rect.x_offset + target.rois[0].rect.width > 100)
                  track_qr_flag = true;
              }
              if(target.rois[0].confidence > 0.8)
              {
                data_ = 4;
                QRcode_flag = true;
              }
            }
            else if (target.type == "P")
            {
              P_flag = true;
              // P_flag = false;
            }
            cnt_target++;
          }
          
          // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected over!");
          /* 执行判断任务逻辑*/
          /*停车*/
          if (recognize_P == true && mission3_flag == false)
          {
            // END_flag = true;
            park();
          }
          if (recognize_P == true && mission3_flag == true && Obstacle_flag == false)
          {
            // if(Obstacle_flag == false)
            // {
            park();
            // }
          }
          /*任务1*/
          if (mission3_flag == false)
          // if (P_flag == false && mission3_flag == false)
          {
            /*二维码跟踪*/
            if (track_qr_flag == true)
            {
              if(Obstacle_flag == false)
                Qrtracking();
            }
            else if (Obstacle_flag == false)
            {
              if (Line_flag == true)
              {
                LineFollowing(targets_msg->targets[line_index_far]);
                if (state_flag != "Follow line without ob")
                {
                  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line without obstacle!");
                  state_flag = "Follow line without ob";
                }
              }
            }
            else
            {
              if (bottom_temp < bottom_threshold_)
              {
                // if (bottom_temp < bottom_threshold_ && recognize_line == true){
                if (Line_flag == true)
                {
                  LineFollowing(targets_msg->targets[line_index_far]);
                  if (state_flag != "Follow line with far ob")
                  {
                    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line with far obstacle!");
                    state_flag = "Follow line with far ob";
                  }
                }
              }
              else
              {
                const auto &target1 = targets_msg->targets[target_index];
                if (target1.rois[0].confidence > confidence_threshold_)
                {
                  ObstaclesAvoiding(target1,targets_msg->targets[line_index_far]);
                }
              }
            }
          }
          /*任务3*/
          else if (mission3_flag == true)
          {
            if (Obstacle_flag == false)
            {
              if (Line_flag == true)
              {
                LineFollowing(targets_msg->targets[line_index_far]);
                if (state_flag != "Follow line without ob")
                {
                  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line without obstacle!");
                  state_flag = "Follow line without ob";
                }
              }
            }
            else
            {
              if (bottom_temp < bottom_threshold_III)
              {
                // if (bottom_temp < bottom_threshold_ && recognize_line == true){
                if (Line_flag == true)
                {
                  LineFollowing(targets_msg->targets[line_index_far]);
                  if (state_flag != "Follow line with far ob")
                  {
                    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line with far obstacle!");
                    state_flag = "Follow line with far ob";
                  }
                }
              }
              else
              {
                const auto &target1 = targets_msg->targets[target_index];
                if (target1.rois[0].confidence > confidence_threshold_)
                {
                  ObstaclesAvoiding(target1,targets_msg->targets[line_index_far]);
                }
              }
            }
          }
          lock.lock();
        }
      }
    }
  }
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &target)
{
  float line_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float line_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
  float temp = line_x - 320.0;
  if ((-20 < line_x) && (line_x < 0))
  {
    temp = -20;
  }
  else if ((line_x > 0) && (line_x < 20))
  {
    temp = 20;
  }
  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = follow_angular_ratio_ * temp / 150.0 * line_y / 224.0;
  if (mission3_flag == false)
  {
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
  twist_msg.linear.x = follow_line_speed;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  if (twist_msg.angular.z > 32.5)
  {
    twist_msg.angular.z = 32.5;
  }
  else if (twist_msg.angular.z < -32.5)
  {
    twist_msg.angular.z = -32.5;
  }
  publisher_->publish(twist_msg);
  avoid_flag = false;
}

void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &ob_target, const ai_msgs::msg::Target &line_target)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  float line_far_x = line_target.rois[0].rect.x_offset + line_target.rois[0].rect.width;
  float angular_z1;
  float angular_z2;
  float angular_z3;
  float avoid_linear_speed1_;
  float avoid_linear_speed2_;
  float avoid_linear_speed3_;
  int tim1_;
  int tim2_;
  // avoid_flag表示是否进行过避障
  if (avoid_flag == false)
  {
    if (state_flag != "Avoiding ob")
    {
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now avoiding obstacle");
      state_flag = "Avoiding ob";
    }

    float ob_x = ob_target.rois[0].rect.x_offset + ob_target.rois[0].rect.width / 2;
    // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of ob_x is: %f", ob_x);
    float ob_edge = ob_target.rois[0].rect.x_offset + ob_target.rois[0].rect.width;
    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of ob_edge is: %f", ob_edge);

    // float temp;
    // float center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
    // center_x = ob_vec.rois[0].rect.x_offset + ob_vec.rois[0].rect.width / 2;sss
    if (line_far_empty == true)
    {
      return;
    }
    float line_edge= line_far.rois[0].rect.x_offset + line_far.rois[0].rect.width;
    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of line_edge is: %f", line_edge);
    float temp = ob_x - 320;
    if ((-temp_lim < temp) && (temp < 0))
    {
      temp = -temp_lim;
    }
    else if ((temp >= 0) && (temp < temp_lim))
    {
      temp = temp_lim;
    }
    // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of temp_lim is: %f", temp);
    // if (recognize_line)
    // {
    //   float x_error = line_far_x - ob_x;
    //   // 认为障碍物在线上
    //   if ((x_error > -ob_line_x) && (x_error < ob_line_x))
    //   {
    //     temp = temp_lim;   // 强制左转
    //   }
    //   else
    //   {
    //     temp = 1.0 * (-x_error);
    //   }
    // }
    // // 只有障碍物
    // else
    // {
    //   temp = ob_x - 320.0;
    // }

    // if ((-temp_lim < temp) && (temp < 0))
    // {
    //   temp = -temp_lim;
    // }
    // else if ((temp >= 0) && (temp < temp_lim))
    // {
    //   temp = temp_lim;
    // }

    // 判断任务一和三
    if (mission3_flag == false) // 任务一
    {
      if (ob_edge > line_edge + ob_offset)
      {
        temp = temp_lim;
      }
      if (temp > 0) // 左转
      {
        tim1_ = time1_l_I_;
        tim2_ = time2_l_I_;
        // tim3_ = time3_l_;
        angular_z1 = avoid_angular_ratio1_l_I_ * 700 / temp;
        angular_z2 = avoid_angular_ratio2_l_I_ * 700 / temp;
        angular_z3 = avoid_angular_ratio3_l_I_ * angular_z1;

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
        angular_z3 = avoid_angular_ratio3_r_I_ * angular_z1;

        avoid_linear_speed1_ = avoid_linear_speed1_r_I_;
        avoid_linear_speed2_ = avoid_linear_speed2_r_I_;
        avoid_linear_speed3_ = avoid_linear_speed3_r_I_;
      }

      // 第一段//////////////////////////////////////////
      while(recognize_ob == true)
      {
        twist_msg.linear.x = avoid_linear_speed1_;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_z1;
        if (twist_msg.angular.z > 32.5)
        {
          twist_msg.angular.z = 32.5;
        }
        else if (twist_msg.angular.z < -32.5)
        {
          twist_msg.angular.z = -32.5;
        }
        publisher_->publish(twist_msg);
        // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of v is: %f", angular_z1);
        // sum_z += angular_z1;
        // z_num++
        // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to avoid obstacle1!");
      }
      
      // 3R-2
      twist_msg.linear.x = avoid_linear_speed2_;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = angular_z2;
      if (twist_msg.angular.z > 32.5)
      {
        twist_msg.angular.z = 32.5;
      }
      else if (twist_msg.angular.z < -32.5)
      {
        twist_msg.angular.z = -32.5;
      }
      publisher_->publish(twist_msg);
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to return!");
      usleep(tim2_ * 1000); // 休眠1000ms
    }
    else // 任务三
    {
      if (ob_edge > line_edge + ob_offset_III)
      {
        temp = temp_lim;
      }
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of temp is: %f", temp);
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of offset is: %f", ob_offset_III);
      
      if (temp > 0) // 左转
      {
        tim1_ = time1_l_III_;
        tim2_ = time2_l_III_;
        // tim3_ = time3_l_;
        angular_z1 = avoid_angular_ratio1_l_III_ * 700 / temp;
        angular_z2 = avoid_angular_ratio2_l_III_ * 700 / temp;
        angular_z3 = avoid_angular_ratio3_l_III_ * angular_z1;

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
        angular_z3 = avoid_angular_ratio3_r_III_ * angular_z1;

        avoid_linear_speed1_ = avoid_linear_speed1_r_III_;
        avoid_linear_speed2_ = avoid_linear_speed2_r_III_;
        avoid_linear_speed3_ = avoid_linear_speed3_r_III_;
      }

      // 第一段
      while(recognize_ob == true)
      {
        twist_msg.linear.x = avoid_linear_speed1_;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_z1;
        if (twist_msg.angular.z > 32.5)
        {
          twist_msg.angular.z = 32.5;
        }
        else if (twist_msg.angular.z < -32.5)
        {
          twist_msg.angular.z = -32.5;
        }
        publisher_->publish(twist_msg);
        // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of v is: %f", angular_z1);
        // sum_z += angular_z1;
        // z_num++
        // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to avoid obstacle1!");
      }

      // 3R-2
      twist_msg.linear.x = avoid_linear_speed2_;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = angular_z2;
      if (twist_msg.angular.z > 32.5)
      {
        twist_msg.angular.z = 32.5;
      }
      else if (twist_msg.angular.z < -32.5)
      {
        twist_msg.angular.z = -32.5;
      }
      publisher_->publish(twist_msg);
      RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Sleep to return!");
      usleep(tim2_ * 1000); // 休眠1000ms
    }

    avoid_flag = true;
    ob2line_flag = true;
    follow_line_speed = avoid_linear_speed3_;
  }

  while (recognize_line != true) // 如果巡线没有传递线的消息，进行姿态调整
  {
    // 根据是否看到线回巡线

    twist_msg.linear.x = avoid_linear_speed3_;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = -angular_z3;
    if (twist_msg.angular.z > 32.5)
    {
      twist_msg.angular.z = 32.5;
    }
    else if (twist_msg.angular.z < -32.5)
    {
      twist_msg.angular.z = -32.5;
    }
    publisher_->publish(twist_msg);
    // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "return!");
    if (angular_z3 > 0)
    {
      if (recognize_qr == true)
      {
        if (QRcode_flag == true)
        {
          twist_msg.linear.x = 0.0;
          twist_msg.linear.y = 0.0;
          twist_msg.linear.z = 0.0;
          twist_msg.angular.x = 0.0;
          twist_msg.angular.y = 0.0;
          twist_msg.angular.z = 0.0;
          publisher_->publish(twist_msg);
          break;
        }
        else
        {
          Qrtracking();
          break;
        }
      }
    }
    else
    {
      if (QRcode_flag == true)
        {
          twist_msg.linear.x = 0.0;
          twist_msg.linear.y = 0.0;
          twist_msg.linear.z = 0.0;
          twist_msg.angular.x = 0.0;
          twist_msg.angular.y = 0.0;
          twist_msg.angular.z = 0.0;
          publisher_->publish(twist_msg);
          break;
        }
    }
    if (recognize_P == true)
    {
      park();
    }
  }
}

void RacingControlNode::Qrtracking()
{
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now tracking the qrcode!");
  if (recognize_qr == false)
  {
    return;
  }
  float line_x = qr_vec.rois[0].rect.x_offset + qr_vec.rois[0].rect.width / 2;
  float line_y = qr_vec.rois[0].rect.y_offset + qr_vec.rois[0].rect.height / 2;
  float temp = line_x - 320.0;
  if ((-20 < line_x) && (line_x < 0))
  {
    temp = -20;
  }
  else if ((line_x > 0) && (line_x < 20))
  {
    temp = 20;
  }
  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = qr_angular_ratio * temp / 150.0 * line_y / 224.0;
  twist_msg.linear.x = QR_speed;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  if (twist_msg.angular.z > 32.5)
  {
    twist_msg.angular.z = 32.5;
  }
  else if (twist_msg.angular.z < -32.5)
  {
    twist_msg.angular.z = -32.5;
  }
  publisher_->publish(twist_msg);
}

void RacingControlNode::park()
{
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Parking!");
  auto twist_msg = geometry_msgs::msg::Twist();
  int end_cnt = 0;
  bool flag = false;
  while(1)
  {
      // auto P_list = ai_msgs::msg::Target();
      // auto P_list1= ai_msgs::msg::Target();
      // P_list = P[0];
      if(recognize_P)
      {
        end_cnt = 0;
        float x = (P_vec.rois[0].rect.x_offset) + 0.5 * (P_vec.rois[0].rect.width);
        float y = (P_vec.rois[0].rect.y_offset) + 0.5 * (P_vec.rois[0].rect.height);
        float bottom = (P_vec.rois[0].rect.y_offset) + (P_vec.rois[0].rect.height);
        if (bottom >= P_bottom_th)
          flag=true;
        float temp = x - 320.0;
        if(x>-20 && x<0)
          temp = -20;
        else if(x>=0 && x<20) 
          temp = 20;
        float angular_z = park_angular_ratio * temp / 150.0 * y / 480.0;
        if (flag)
          twist_msg.linear.x = stop_linear_vel;
        else
          twist_msg.linear.x = stop_linear_vel_close;

        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = -angular_z;
        if (twist_msg.angular.z > 32.5)
        {
          twist_msg.angular.z = 32.5;
        }
        else if (twist_msg.angular.z < -32.5)
        {
          twist_msg.angular.z = -32.5;
        }
        publisher_->publish(twist_msg);
      }
      else 
      {
        if (flag)
        {
          twist_msg.linear.x = stop_linear_vel;
          twist_msg.linear.y = 0.0;
          twist_msg.linear.z = 0.0;
          twist_msg.angular.x = 0.0;
          twist_msg.angular.y = 0.0;
          twist_msg.angular.z = 0.0;
          publisher_->publish(twist_msg);
          end_cnt += 1;
        }
      }
      if (end_cnt >= stop_cnt_th)
        break;
  }
  while(1)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    publisher_->publish(twist_msg);
  }
}
    

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
