# 源码根文件夹
## face_tracking/
这是存储面捕追踪算法相关的文件夹。目前仅做了眼部追踪。后续可以加入更多。
## servo_control/
这是存储舵机控制（滤波、插值映射、舵机驱动）相关模块的文件夹。slots.py是拓展槽位的入口。
## pipeline_manager.py
这是纯逻辑模块，管理数据流。
## gui.py
这是最终的用户入口。源码运行时直接python3 gui.py就可以打开欢迎页面。