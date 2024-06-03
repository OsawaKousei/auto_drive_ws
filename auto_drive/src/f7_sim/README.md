# f7_sim

## Getting started

##### 使い方の簡単な説明

1.
ros2 launch f7_sim position_move_test.py
で起動する

2.
rqtのcmd_posに絶対座標(相対座標(※後述))の位置を入れるとその地点に向かって直進する

##### 諸設定

- x軸,y軸,yaw方向のpid parametersは
auto_drive_ws/auto_drive/src/f7_sim/include/f7_sim_node.hpp
で変更できる

- (※)auto_drive_ws/auto_drive/src/f7_sim/include/f7_sim_node.hpp
の「#define ABS_COORDINATE 1」を0にするとcmd_posが相対座標指定になる
（「馬の前にニンジンをぶら下げる状態」になる（？））
←
f7_sim外で追従させたいときに使用する

- auto_drive_ws/auto_drive/src/f7_sim/include/noisy_odom_node.hpp
の「#define DISABLE_NOISE 1」を0にするとodometryにノイズが乗るようになる

##### 問題点

- anti-windup処理がうまくいかないときがあり、I項が不安定になりがち




