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
```
Navigate to http://127.0.0.1:8088/cam.mjpg in the browser (Chrome works best)

```
source catkin_ws/devel/setup.bash
rostopic echo /roboy/cognition/vision/visible_face_names
```
