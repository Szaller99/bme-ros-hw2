# Kuka iisy1300 palletizing demo with moveit2

## Introduction
This repository contain code for BME Vik IIT x Kuka hungária homework code of a palletizing simulation.

## Getting started

- Install dependecies:
```sudo apt install -y libnanopb-dev libgrpc++-dev ros-humble-ros2-control ros-humble-ros2-controller```

- Have ```ros2```
- Have ```moveit2``` package installed
- Have ```kuka_drivers``` package installed from github.com/kroshu

- Source the builded packages, and build this one with
```colcon build```

## Running simulation

After building the package: 

1. Launch it with:

```
ros2 launch iiqka_moveit_palletizing moveit_palletizing.launch.py
```

2. Activate robot_manager node:

```
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```

3. Run the code:

```
ros2 run iiqka_moveit_palletizing iiqka_moveit_palletizing
```

4. Run the simulation on ```RViz```