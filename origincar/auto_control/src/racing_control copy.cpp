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

std::string state_flag;
int foxglove_data;
bool mission2_flag = false;
bool mission3_flag = false;
bool line_flag = false;
bool ob_flag = false;
bool QRcode_flag = false;     // 满足置信度才真
bool QR_flag = false;         // 看到就是真
bool P_flag = false;
int bottom_temp_ob_close = 0;
int bottom_temp_ob_far = 480;
int ob_x_close = 0;
int ob_y_close = 0;
int ob_x_far = 0;
int ob_y_far = 0;
int QR_x;
int QR_y;
int P_x = 0;
int P_y = 0;
int P_bottom = 0;
int P_cnt = 0;

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

  this->declare_parameter<float>("confidence_threshold_", confidence_threshold_);
  this->get_parameter<float>("confidence_threshold_", confidence_threshold_);
  
  this->declare_parameter<int>("ob_bottom_th_in_I", ob_bottom_th_in_I);
  this->get_parameter<int>("ob_bottom_th_in_I", ob_bottom_th_in_I);
  this->declare_parameter<int>("ob_bottom_th_in_III", ob_bottom_th_in_III);
  this->get_parameter<int>("ob_bottom_th_in_III", ob_bottom_th_in_III);

  this->declare_parameter<float>("re_vel", re_vel);
  this->get_parameter<float>("re_vel", re_vel);
  this->declare_parameter<int>("re_tim", re_tim);
  this->get_parameter<int>("re_tim", re_tim);

  this->declare_parameter<float>("follow_linear_speed_I", follow_linear_speed_I);
  this->get_parameter<float>("follow_linear_speed_I", follow_linear_speed_I);
  this->declare_parameter<float>("follow_angular_ratio_I", follow_angular_ratio_I);
  this->get_parameter<float>("follow_angular_ratio_I", follow_angular_ratio_I);
  this->declare_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
  this->get_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
  this->declare_parameter<float>("follow_angular_ratio_III", follow_angular_ratio_III);
  this->get_parameter<float>("follow_angular_ratio_III", follow_angular_ratio_III);

  this->declare_parameter<int>("line_ob_x_th", line_ob_x_th);
  this->get_parameter<int>("line_ob_x_th", line_ob_x_th);
  
  this->declare_parameter<float>("avoid_linear_speed_I", avoid_linear_speed_I);
  this->get_parameter<float>("avoid_linear_speed_I", avoid_linear_speed_I);
  this->declare_parameter<float>("avoid_angular_ratio_I", avoid_angular_ratio_I);
  this->get_parameter<float>("avoid_angular_ratio_I", avoid_angular_ratio_I);
  this->declare_parameter<int>("ob_bottom_th_I", ob_bottom_th_I);
  this->get_parameter<int>("ob_bottom_th_I", ob_bottom_th_I);
  this->declare_parameter<int>("tim_I", tim_I);
  this->get_parameter<int>("tim_I", tim_I);
  this->declare_parameter<float>("avoid_linear_speed_III", avoid_linear_speed_III);
  this->get_parameter<float>("avoid_linear_speed_III", avoid_linear_speed_III);
  this->declare_parameter<float>("avoid_angular_ratio_III", avoid_angular_ratio_III);
  this->get_parameter<float>("avoid_angular_ratio_III", avoid_angular_ratio_III);
  this->declare_parameter<int>("ob_bottom_th_III", ob_bottom_th_III);
  this->get_parameter<int>("ob_bottom_th_III", ob_bottom_th_III);
  this->declare_parameter<int>("tim_III", tim_III);
  this->get_parameter<int>("tim_III", tim_III);

  this->declare_parameter<int>("temp_lim", temp_lim);
  this->get_parameter<int>("temp_lim", temp_lim);
  
  this->declare_parameter<int>("modify_cnt_th", modify_cnt_th);
  this->get_parameter<int>("modify_cnt_th", modify_cnt_th);
  this->declare_parameter<float>("modify_ratio", modify_ratio);
  this->get_parameter<float>("modify_ratio", modify_ratio);

  this->declare_parameter<int>("P_cnt_th", P_cnt_th);
  this->get_parameter<int>("P_cnt_th", P_cnt_th);
  this->declare_parameter<int>("line_cnt_th", line_cnt_th);
  this->get_parameter<int>("line_cnt_th", line_cnt_th);
  this->declare_parameter<int>("P_bottom_th", P_bottom_th);
  this->get_parameter<int>("P_bottom_th", P_bottom_th);

  this->declare_parameter<float>("park_angular_ratio", park_angular_ratio);
  this->get_parameter<float>("park_angular_ratio", park_angular_ratio);
  this->declare_parameter<float>("stop_linear_vel", stop_linear_vel);
  this->get_parameter<float>("stop_linear_vel", stop_linear_vel);
  this->declare_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
  this->get_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
 
  this->declare_parameter<int>("stop_cnt_th", stop_cnt_th);
  this->get_parameter<int>("stop_cnt_th", stop_cnt_th);
  
  this->declare_parameter<float>("QR_follow_raio", QR_follow_raio);
  this->get_parameter<float>("QR_follow_raio", QR_follow_raio);
  this->declare_parameter<float>("QR_follow_vel", QR_follow_vel);
  this->get_parameter<float>("QR_follow_vel", QR_follow_vel);

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
    P_x = 0;
    P_y = 0;
    P_bottom = 0;
    line_flag = false;
    QR_flag = false;
    P_flag = false;
    ob_flag = false;
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
          line_flag = true;
        }
        else if (target.type == "obstacle")
        {
          if (target.rois[0].confidence > confidence_threshold_)
          {
            // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected obstacle!");
            int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
            int ob_x = target.rois[0].rect.x_offset + 0.5 * target.rois[0].rect.width;
            int ob_y = target.rois[0].rect.y_offset + 0.5 * target.rois[0].rect.height;
            // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "The value of bottom is: %d", bottom);
            if (bottom > bottom_temp_ob_close)
            {
              bottom_temp_ob_close = bottom;
              ob_x_close = ob_x;
              ob_y_close = ob_y;
            }
            if (bottom < bottom_temp_ob_far)
            {
              bottom_temp_ob_far = bottom;
              ob_x_far = ob_x;
              ob_y_far = ob_y;
            }
            ob_flag = true;
          }
        }
        else if ((target.type == "clock") || (target.type == "anticlock"))
        {
          QR_flag = true;
          QR_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
          QR_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
          if (target.rois[0].confidence > 0.8)
          {
            QRcode_flag = true;
          }
        }
        else if (target.type == "P")
        {
          // if( target.rois[0].confidence > 0.7) 
          // {
          P_flag = true;
          // P_vec = target;
          P_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
          P_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
          P_bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
          // }
        }
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
    int cnt_target = 0;
    int ob_bottom_th_in;
    int bottom_temp_line_close = 0;
    int bottom_temp_line_far = 480;
    int line_index_close;
    int line_index_far;

    this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);
    this->get_parameter<float>("confidence_threshold_", confidence_threshold_);
    this->get_parameter<int>("ob_bottom_th_in_I", ob_bottom_th_in_I);
    this->get_parameter<int>("ob_bottom_th_in_III", ob_bottom_th_in_III);
    this->get_parameter<int>("re_tim", re_tim);
    this->get_parameter<float>("re_vel", re_vel);
    this->get_parameter<float>("follow_linear_speed_I", follow_linear_speed_I);
    this->get_parameter<float>("follow_angular_ratio_I", follow_angular_ratio_I);
    this->get_parameter<float>("follow_linear_speed_III", follow_linear_speed_III);
    this->get_parameter<float>("follow_angular_ratio_III", follow_angular_ratio_III);
    this->get_parameter<int>("line_ob_x_th", line_ob_x_th);
    this->get_parameter<float>("avoid_linear_speed_I", avoid_linear_speed_I);
    this->get_parameter<float>("avoid_angular_ratio_I", avoid_angular_ratio_I);
    this->get_parameter<int>("ob_bottom_th_I", ob_bottom_th_I);
    this->get_parameter<int>("tim_I", tim_I);
    this->get_parameter<float>("avoid_linear_speed_III", avoid_linear_speed_III);
    this->get_parameter<float>("avoid_angular_ratio_III", avoid_angular_ratio_III);
    this->get_parameter<int>("ob_bottom_th_III", ob_bottom_th_III);
    this->get_parameter<int>("tim_III", tim_III);
    this->get_parameter<int>("temp_lim", temp_lim);
    this->get_parameter<int>("modify_cnt_th", modify_cnt_th);
    this->get_parameter<float>("modify_ratio", modify_ratio);
    this->get_parameter<int>("P_cnt_th", P_cnt_th);
    this->get_parameter<int>("line_cnt_th", line_cnt_th);
    this->get_parameter<int>("P_bottom_th", P_bottom_th);
    this->get_parameter<float>("park_angular_ratio", park_angular_ratio);
    this->get_parameter<float>("stop_linear_vel", stop_linear_vel);
    this->get_parameter<float>("stop_linear_vel_close", stop_linear_vel_close);
    this->get_parameter<int>("stop_cnt_th", stop_cnt_th);
    this->get_parameter<float>("QR_follow_raio", QR_follow_raio);
    this->get_parameter<float>("QR_follow_vel", QR_follow_vel);


    // 已经进入任务三，才开始检测有没有P
    if (mission3_flag == true)
    {
      ob_bottom_th_in = ob_bottom_th_in_III;

      if ((P_flag == true) && (ob_flag == false))
      {
        P_cnt += 1;
      }
      if (P_cnt >= P_cnt_th)
      {
        park();
      }
    }
    else
    {
      ob_bottom_th_in = ob_bottom_th_in_I;
    }

    // 正在进行任务二
    if (mission2_flag == true)
    {
      // if (cnt_ms2 < 1)
      // {
      //   RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Mission2");
      //   cnt_ms2++;
      // }
    }
    // 没有进行任务二
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
            
          }
          else if (target.type == "line")
          {
            // RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Detected line!");
              line_flag = true;
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
            if(target.rois[0].confidence > 0.8)
            {
              data_ = 3;
              QRcode_flag = true;
            }
          }
          else if (target.type == "anticlock")
          {
            if(target.rois[0].confidence > 0.8)
            {
              data_ = 4;
              QRcode_flag = true;
            }
          }
          else if (target.type == "P")
          {

          }
          cnt_target++;
        }


        // 识别到了二维码【符合置信度】
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
        // 没有看到二维码
        else
        {
          // 没有障碍物
          if (ob_flag == false)
          {
            if (QR_flag == true)
            {
              // QR_follow(QR_x,QR_y);
            }
            // 有线、没障碍物
            else if ((line_flag == true) && (QR_flag == false))
            {
              LineFollowing(targets_msg->targets[line_index_close]);
              if (state_flag != "Follow line without ob")
              {
                RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now following line without obstacle!");
                state_flag = "Follow line without ob";
              }
            }
          }
          // 有障碍物
          else
          {
            if (bottom_temp_ob_close <= ob_bottom_th_in)
            {
              LineFollowing(targets_msg->targets[line_index_close]);
            }
            else
            {
              ObstaclesAvoiding(targets_msg->targets[line_index_far]);
            }
          }
        }

        lock.lock();
      }
      

      
    }
  }
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &target)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  float line_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float line_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
  float temp = line_x - 320.0;
  float angular_z;
  float follow_linear_speed;
  float follow_angular_ratio;

  if (mission3_flag == false)
  {
    follow_linear_speed = follow_linear_speed_I;
    follow_angular_ratio = follow_angular_ratio_I;
  }
  else
  {
    follow_linear_speed = follow_linear_speed_III;
    follow_angular_ratio = follow_angular_ratio_III;
  }

  if ((-20 < line_x) && (line_x < 0))
  {
    temp = -20;
  }
  else if ((line_x > 0) && (line_x < 20))
  {
    temp = 20;
  }
  
  angular_z = follow_angular_ratio * temp / 150.0 * line_y / 224.0;
  
  twist_msg.linear.x = follow_linear_speed;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
  // avoid_flag = false;
}

void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &target)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  float avoid_linear_speed;
  float avoid_angular_ratio;
  float angular_z;
  int ob_bottom_th;
  int temp;
  int modify_cnt = 0;
  int P_cnt = 0;
  int line_cnt = 0;
  int sum_z = 0;
  int z_num = 0;
  int tim;

  // 日志输出
  if (state_flag != "Avoiding ob")
  {
    RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Now avoiding obstacle");
    state_flag = "Avoiding ob";
  }
  
  // 参数判断
  if (mission3_flag == true)
  {
    avoid_linear_speed = avoid_linear_speed_III;
    avoid_angular_ratio = avoid_angular_ratio_III;
    ob_bottom_th = ob_bottom_th_III;
    tim = tim_III;
  }
  else
  {
    avoid_linear_speed = avoid_linear_speed_I;
    avoid_angular_ratio = avoid_angular_ratio_I;
    ob_bottom_th = ob_bottom_th_I;
    tim = tim_I;
  }

  // 先减速
  twist_msg.linear.x = avoid_linear_speed;
  publisher_->publish(twist_msg);

  // 第一段
  while(1)
  {
    // 看到障碍物
    if (ob_flag == true)
    {
      // 最近的障碍物较近
      if (bottom_temp_ob_close < ob_bottom_th)
      {
        /* 计算temp值 */ 
        // 有线、有障碍物
        if (line_flag == true)
        {
          int line_x_far = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
          int line_y_far = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
          int line_ob_error = line_x_far - ob_x_close;
          if (line_ob_error < 0)
          {
            line_ob_error = -line_ob_error;
          }
          // 认为障碍物在线的中间
          if (line_ob_error < line_ob_x_th)
          {
            temp = temp_lim;
          }
          else
          {
            temp = -line_ob_error;
          }
        }
        // 只有障碍物
        else
        {
          temp = ob_x_close - 320;
        }
        /* temp值限幅 */
        if ((temp > -temp_lim) && (temp <= 0))
        {
          temp = -temp_lim;
        }
        else if ((temp > 0) && (temp < temp_lim))
        {
          temp = temp_lim;
        }
        /* 计算转角z */
        angular_z = avoid_angular_ratio * 700 / temp;
        /* 发布避障第一段 */
        twist_msg.linear.x = avoid_linear_speed;
        twist_msg.angular.z = angular_z;
        publisher_->publish(twist_msg);
        sum_z += angular_z;
        z_num += 1;
      }
      // 最近的障碍物较远
      else
      {
        modify_cnt += 1;
      }
    }
    // 看不到障碍物
    else
    {
      modify_cnt += 1;
    }

    if (modify_cnt >= modify_cnt_th)
    {
      break;
    }
  }

  // 第二段
  twist_msg.linear.x = avoid_linear_speed;
  twist_msg.angular.z = 0.0;
  publisher_->publish(twist_msg);
  usleep(tim * 1000);

  // 第三段
  while(1)
  {
    // 任务一
    if (mission3_flag == false)
    {
      // 满足巡二维码条件
      if ((QR_flag == true) && (mission2_flag == false) && (ob_flag == false))
      {
        // QR_follow(QR_x,QR_y);
        break;
      }
      // 不满足巡二维码条件
      else
      {
        angular_z = - modify_ratio * (sum_z / z_num);
        twist_msg.linear.x = avoid_linear_speed;
        twist_msg.angular.z = angular_z;
        publisher_->publish(twist_msg);
      }
    }
    // 任务三
    else
    {
      if ((P_flag == true) && (ob_flag == false))
      {
        P_cnt += 1;
      }
      if (line_flag == true)
      {
        line_cnt += 1;
      }

      if (P_cnt >= P_cnt_th)
      {
        park();
      }
      else if (line_cnt >= line_cnt_th)
      {
        break;
      }
      else
      {
        angular_z = - modify_ratio * (sum_z / z_num);
        twist_msg.linear.x = avoid_linear_speed;
        twist_msg.angular.z = angular_z;
        publisher_->publish(twist_msg);
      }
    }
  }
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
    if(P_flag)
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
        twist_msg.angular.z = -angular_z;
      }
      // 未靠近过P，且看到P，应该以大速度靠近
      else
      {
        twist_msg.linear.x = stop_linear_vel_close;
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
    
void RacingControlNode::QR_follow(int x, int y)
{
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "Following QRcode!");
  auto twist_msg = geometry_msgs::msg::Twist();
  float temp = x - 320.0;

  if(temp>-20 && temp<=0)
    temp = -20;
  else if(temp>0 && temp<20) 
    temp = 20;

  float angular_z = QR_follow_raio * temp / 150.0 * y / 480.0;

  twist_msg.linear.x = QR_follow_vel;
  twist_msg.angular.z = -angular_z;
  publisher_->publish(twist_msg);
}

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
