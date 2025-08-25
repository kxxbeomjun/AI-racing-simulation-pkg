# ERP42 Simulator

Multiple ERP42 vehicle simulator (Based on Gazebo Sim)

## TODO

[TODO Board (Notion)](https://www.notion.so/59bdfdc028a84c6ebcc4a95b785d6802?v=1305e9574541454bb6cb67a3c8cc4128&p=043e1a65f76d4438a363625f85cc1f7c&pm=s)


## How To Use

### Build and requirements

```bash
# Download and update gitsubmodule
$ cd (your ros workspace)/src
$ git clone https://github.com/Yonsei-AI-Racing/erp42_simulator.git
$ cd erp42_simulator
$ git submodule init
$ git submodule update

# Install dependencies
$ rosdep install --from-paths . --ignore-src -r -y

# Build our packages
$ cd (your ros workspace)
$ catkin_make
```

### Run Simulator (Single vehicle)

```bash
$ roslaunch erp42_vehicle_gazebo erp42_gazebo_sim_single.launch
$ roslaunch erp42_vehicle_gazebo erp42_navigation_demo.launch
```

### Run Simulator (Multiple vehicle)

```bash
$ roslaunch erp42_vehicle_gazebo erp42_gazebo_sim_multi.launch
$ roslaunch erp42_vehicle_gazebo erp42_navigation_multi.launch
```

### Run Simulator with movebase_follwer (Multiple vehicle)
```bash
$ roslaunch erp42_vehicle_gazebo erp42_navigation_multi.launch autostart:=true
```

### Run Simulator with movebase_follwer (Multiple vehicle, input delay 추가)
```bash
$ roslaunch erp42_vehicle_gazebo erp42_navigation_multi_w_delay.launch autostart:=true
```

## 그 외 기타 설명

### 순위 별 속도 관련 
referee_server.py 노드에서 각 차량의 순위별로 낼 수 있는 최대 속도를 1위는 5.0m/s, 그 외 후순위 차량에 대해서는 4.0m/s로 제한되어 있음. 해당 노드에서는 아래 파라미터를 순위에 따라 세팅.
```bash
(namespace)/move_base/RegulatedPurePursuitController/max_allowed_velocity 
```
이후 regulated_pure_pursuit_controller에서 dynamic_reconfigure로 파라미터를 세팅한 뒤 [applyConstraints 함수 마지막](https://github.com/Yonsei-AI-Racing/erp42_simulator/blob/4b3ab224bdb26972e309f7cbadde3d039cc45c03/erp42_navigation_demo/regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp#L459)에 해당 속도를 적용.
Planner 변경하여 사용시 해당 코드 포함해서 작업

[ERP42 simulator trouble shooting/issues](./docs/troubleshooting_issues.md)

[ERP42 simulator 패키지 구조](./docs/package_tree.md)

[gitsubmodule 관련 사용 가이드](./docs/howtouse_gitsubmodule.md)
