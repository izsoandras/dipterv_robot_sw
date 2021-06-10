﻿# Diploma project - software running on the robots

This is the software running on the little 3 wheeled omnirobots. The goal is to receive a path consisting of a polyline from a computer through MQTT and naviage through it.
The position will be estimated by the IMU on the robot, and from a camera overlooking the working area. The data from the camera and the IMU will be fused through a Kalman filter. In each endpoint the robot will measure the distances around itself and send it to the computer, which will be building a map from these data.
