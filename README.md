# 🤖 Autonomous Gas Detection Robot using ROS 2 and Gazebo

## 📌 Project Overview

This project implements an autonomous mobile robot for gas detection using **ROS 2 Humble** and **Gazebo Classic**. The robot monitors simulated gas concentration, searches for elevated gas levels, identifies a danger zone, and displays sensor readings and alerts through a monitoring dashboard.

## 🎯 Objectives

- Simulate gas sensor readings, including gas concentration, temperature, and humidity.
- Enable autonomous robot movement during gas-source searching.
- Detect critical gas concentration levels.
- Report the estimated gas-source location.
- Visualize the detected source in Gazebo.
- Display live sensor data and alerts through a dashboard.

## ✨ Features

- Gas-gradient-based searching behavior.
- Custom ROS 2 gas sensor message.
- Robot odometry and velocity command monitoring.
- Critical gas detection at **500 ppm**.
- Danger-zone coordinate publishing.
- Red sphere visualization of the detected gas source.
- Tkinter dashboard with gas status alerts.
- ROS 2 bag recording for experiment analysis.

## 🛠️ Technologies Used

- Ubuntu
- ROS 2 Humble
- Gazebo Classic
- Python 3
- ROS 2 custom messages
- Tkinter
- Matplotlib
- Git and GitHub

## 📡 ROS 2 Topics

| Topic | Purpose |
|---|---|
| `/gas_sensor` | Gas concentration, temperature, humidity, and sensor health |
| `/odom` | Robot position and orientation |
| `/cmd_vel` | Robot velocity commands |
| `/danger_zone` | Estimated gas-source coordinates |
| `/visualization_marker` | Visualization markers |

## 🧭 System Workflow

1. Gazebo launches the simulated robot and environment.
2. The gas simulator publishes sensor readings.
3. The decision-making node uses sensor readings to control robot movement.
4. When the critical gas threshold is reached, the robot stops and reports the estimated source location.
5. A red sphere represents the detected gas source in Gazebo.
6. The monitoring dashboard displays sensor values, robot position, and alerts.
7. ROS 2 bag files can be recorded for offline experiment analysis.

## 🚀 Build the Workspace

Install ROS 2 Humble and the required project dependencies first.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Launch the simulation using the launch command configured for your project.

## 📊 Experiment Analysis

ROS 2 bag recordings can be used to analyze gas concentration, robot trajectory, odometry, and danger-zone data. Graphs can be generated to evaluate the robot's behavior during simulated experiments.

## ⚠️ Limitations

- Gas sensing and detection are simulated rather than validated with a physical gas sensor.
- Source localization accuracy depends on the robot's movement, sensor model, and decision-making logic.
- Real-world deployment would require hardware integration and safety testing.

## 🔮 Future Improvements

- Integrate a physical gas sensor.
- Test on a real mobile robot.
- Improve gas-source localization.
- Add persistent detection records and experiment summaries.
- Evaluate performance across different simulated environments.

## 👨‍💻 Author

**Anikesh**

GitHub: [unofficialank121-glitch](https://github.com/unofficialank121-glitch)

## 📄 License

A license can be added when the project's distribution terms are decided.
