# !/bin/bash

ros2 run qrcoder racing_control \
--ros-args \
-p follow_angular_ratio:=1.0 \
-p follow_linear_speed:=0.4 \
-p ob_bottom_threshold:=320 \
-p ob_bottom_threshold_in:=290 \
-p avoid_linear_speed:=0.2 \
-p sleep_time:=0.3 \
-p modify_ratio:=0.8 \
-p QR_cnt_th:=1000 \
-p line_cnt_threshold:=1500 \
-p P_bottom_th:=400 \
-p park_s_close:=0.3 \
-p park_s_stop:=0.1 \
-p park_angular_ratio:=4.0 \
-p end_cnt_th:=2000