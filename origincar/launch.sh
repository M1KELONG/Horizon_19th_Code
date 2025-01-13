# !/bin/bash

ros2 launch origincar_bringup \
newracing_control.launch.py \
receive_t:=0.04 \
follow_angular_ratio_I:=3.0 \
follow_linear_speed_I:=0.5 \
ob_bottom_th_I:=290 \
ob_bottom_th_in_I:=280 \
avoid_angular_ratio_I:=2.0 \
avoid_linear_speed_I:=0.2 \
modify_cnt_th_I:=10 \
sleep_time_I:=0.3 \
modify_ratio_I:=0.5 \
line_cnt_th_I:=500 \
follow_angular_ratio_III:=1.5 \
follow_linear_speed_III:=0.3 \
ob_bottom_th_III:=320 \
ob_bottom_th_in_III:=300 \
avoid_angular_ratio_III:=0.75 \
avoid_linear_speed_III:=0.2 \
modify_cnt_th_III:=25 \
sleep_time_III:=0.3 \
modify_ratio_III:=0.6 \
line_cnt_th_III:=1500 \
QR_cnt_th:=1000 \
line_ob_x_th:=20 \
P_bottom_th:=400 \
park_s_close:=0.3 \
park_s_stop:=0.05 \
park_angular_ratio:=4.0 \
end_cnt_th:=500