# ERP42 Gazebo Simulator

## How to Use

### 1. Build

```bash
$ cd (your ros workspace)
$ catkin_make
```

### 2. Launch

Gazebo는 연산량이 높은 시뮬레이터로, GUI를 키면 매우 무거워짐.
때문에 현재 GUI를 default로 꺼두고, Rviz로 차량을 확인함.

**launch with guiless**

```bash
$ roslaunch erp42_vehicle_gasebo erp42_gazebo_sim.launch
```

**launch with gui**

```bash
$ roslaunch erp42_vehicle_gasebo erp42_gazebo_sim.launch gui:=true
```