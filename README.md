# face_oracle
Requires:
- ROS melodic or kinetic

```
pip install --user face_recognition websocket_client Pillow

cd catkin_ws/src
git clone https://github.com/Roboy/roboy_communication.git
git clone https://github.com/Roboy/face_oracle.git

cd catkin_ws
catkin_make
source catkin_ws/devel/setup.bash

roscore&

cd catkin_ws/src/face_oracle
python webcam_video_processor.py
