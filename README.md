# ERP42 Simulator

Multiple ERP42 vehicle simulator (Based on Gazebo Sim)

## TODO
- [ ] virtual frame braodcaster gitsubmodule로 교체

- [ ] python 기반 simulation control GUI 노드 작성
- [ ] python rich library 기반으로 GUI 교체

## How To Use

### Build and requirements

```bash
$ cd (your ros workspace)
$ catkin_make
```

### Run Simulator

**launch with renderless**

```bash
$ roslaunch erp42_vehicle_gazebo erp42_gazebo_sim.launch
```

**launch with rendering**

```bash
$ roslaunch erp42_vehicle_gazebo erp42_gazebo_sim.launch gui:=true
```

**launch demo navigation**

tf 문제가 해결되지 않아, 현재는 정상 동작하지 않음.
```bash
$ roslaunch erp42_vehicle_gazebo erp42_navigation_demo.launch
```

### Control Simulation

```bash
작업중 (나성재 연구원 연락)
```

## Package tree

[erp42_gazebo](erp42_gazebo)
> Gazebo simulator, vehicle/world model, gazebo controller

[erp42_navigation](erp42_navigation)
> Demo package for gazebo erp42 simulation

[virtual_frame_broadcaster](virtual_frame_broadcaster)
> Tools for 