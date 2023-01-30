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

**launch with renderless**

```bash
$ roslaunch erp42_vehicle_gazebo erp42_gazebo_sim_multi.launch
```

**launch demo navigation**

```bash
$ roslaunch erp42_vehicle_gazebo erp42_navigation_multi.launch
```

## 그 외 기타 설명

[ERP42 simulator trouble shooting/issues](./docs/troubleshooting_issues.md)

[ERP42 simulator 패키지 구조](./docs/package_tree.md)

[gitsubmodule 관련 사용 가이드](./docs/howtouse_gitsubmodule.md)
