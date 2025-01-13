# !/bin/bash

ros2 launch only_line_follow \
racing_control.launch.py \
END_CNT_threshold:=1 \
line_cnt_th:=2 \
recover_cnt:=25 \
temp_lim:=10 \
ob_x_error_l_th:=100 \
ob_x_error_r_th:=150 \
follow_angular_ratio_I:=-0.49 \
follow_linear_speed:=0.7 \
bottom_threshold:=355 \
QR_confidence_th:=0.95 \
QR_follow_vel:=0.6 \
QR_follow_ratio:=0.4 \
QR_cnt_th:=15 \
QR_x_th:=260 \
ob_QR_th:=60 \
re_vel:=0.35 \
re_tim:=0 \
follow_angular_ratio_III:=-0.5 \
follow_linear_speed_III:=0.7 \
bottom_threshold_III:=364 \
P_confidence_th:=0.5 \
recognize_P_conf_th:=0.5 \
P_cnt_th:=5 \
park_angular_ratio:=1.0 \
stop_linear_vel_close:=0.7 \
stop_linear_vel:=0.3 \
stop_cnt_th:=5000