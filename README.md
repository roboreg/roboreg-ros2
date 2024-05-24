# ROS 2 Roboreg
[![License: CC BY-NC 4.0](https://licensebuttons.net/l/by-nc/4.0/80x15.png)](https://github.com/lbr-stack/ros2_roboreg?tab=License-1-ov-file#readme)
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

ROS 2 integration for [roboreg](https://github.com/lbr-stack/roboreg).

## Table of Contents
* [Installation](#installation)
* [Nodes](#nodes)
    * [Roboreg Server](#roboreg-server)
    * [Autoreg](#autoreg)
    * [Static TF Broadcaster](#static-tf-broadcaster)

## Installation
- Install [roboreg](https://github.com/lbr-stack/roboreg):
```shell
pip3 install git+https://github.com/lbr-stack/roboreg.git
```
- Build this wrapper
```shell
mkdir -p lbr-stack/src && cd lbr-stack
git clone https://github.com/lbr-stack/roboreg.git src/ros2_roboreg
colcon build --symlink-install
```

## Nodes
### Roboreg Server
```shell
ros2 launch ros2_roboreg server.launch.py -s
```

#### Subscriped Topics
* `image_rect_color`
* `camera_info` 
* `joint_states`
* `point_cloud/cloud_registered`
* `robot_description`

#### Services
* `~/collect_sample`
* `~/register`
* `~/export/samples`
* `~/export/transform`
* `~/import/transform`
* `~/broadcast_transform`

### Autoreg
Utility node for executing trajectory via `ros2_control` and collecting samples via `ros2_roboreg`.

```shell
ros2 run ros2_roboreg autoreg --help
```

### Static TF Broadcaster
Utility node for publishing static transform as acquired through `ros2_roboreg`.

```shell
ros2 run ros2_roboreg --help
```

## Acknowledgements
### Organizations and Grants
We would further like to acknowledge following supporters:

| Logo | Notes |
|:--:|:---|
| <img src="https://medicalengineering.org.uk/wp-content/themes/aalto-child/_assets/images/medicalengineering-logo.svg" alt="wellcome" width="150" align="left">  | This work was supported by core and project funding from the Wellcome/EPSRC [WT203148/Z/16/Z; NS/A000049/1; WT101957; NS/A000027/1]. |
| <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b7/Flag_of_Europe.svg/1920px-Flag_of_Europe.svg.png" alt="eu_flag" width="150" align="left"> | This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 101016985 (FAROS project). |
| <img src="https://rvim.online/author/avatar_hu8970a6942005977dc117387facf47a75_62303_270x270_fill_lanczos_center_2.png" alt="RViMLab" width="150" align="left"> | Built at [RViMLab](https://rvim.online/). |
| <img src="https://avatars.githubusercontent.com/u/75276868?s=200&v=4" alt="King's College London" width="150" align="left"> | Built at [CAI4CAI](https://cai4cai.ml/). |
| <img src="https://upload.wikimedia.org/wikipedia/commons/1/14/King%27s_College_London_logo.svg" alt="King's College London" width="150" align="left"> | Built at [King's College London](https://www.kcl.ac.uk/). |
