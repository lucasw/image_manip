# image_manip

ros image manipulation nodelets.
Successor project to https://github.com/lucasw/vimjay but cleaned up and with ros2.

## Stop Motion Animation

```
cd ~/colcon_ws/src
git clone -b bouncy https://github.com/lucasw/v4l2ucp
git clone -b internal_pub_sub https://github.com/lucasw/usb_cam
git clone https://github.com/lucasw/imgui_ros
cd imgui_ros/imgui_ros
git clone https://github.com/ocornut/imgui.git
cd ../../
git clone https://github.com/lucasw/image_manip
cd ..
colcon build --packages-select v4l2ucp usb_cam imgui_ros image_manip
source install/setup.bash
ros2 launch image_manip stop_motion_launch.py
```

## RotoZoom

[![RotoZoom](https://img.youtube.com/vi/2vLw7mZvy1M/0.jpg)](https://www.youtube.com/watch?v=2vLw7mZvy1M)

## Infinite Impulse Response

[![IIR on video](https://img.youtube.com/vi/zTzyNL8vQME/0.jpg)](https://www.youtube.com/watch?v=zTzyNL8vQME)

# Related projects

## Noise Image Generation

https://github.com/lucasw/open_simplex_noise_ros

## Screengrab

Capture an image of the screen and publish it as a ros image:
https://github.com/lucasw/screen_grab
