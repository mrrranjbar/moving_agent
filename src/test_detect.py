#!/usr/bin/env python3

import subprocess
import os
img_size = (320,240)
conf = 0.65
subprocess.call(["python3","/home/mohammad/catkin_ws/src/moving_agent/src/detect.py", "--weights", "/home/mohammad/catkin_ws/src/moving_agent/src/best.pt", "--conf", str(conf), "--source", "/home/mohammad/catkin_ws/src/moving_agent/src/images/WIN_20230515_17_22_01_Pro_mp4-21_jpg.rf.1fe1f2a9aed20a5fde50c7946bda9bad.jpg","--img-size", str(img_size[0])
])