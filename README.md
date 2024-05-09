# ROS 2 Roboreg
ROS 2 integration for [roboreg](https://github.com/lbr-stack/roboreg).

## Table of Content

* [Nodes](#nodes)
    * [Roboreg Server](#roboreg-server)
    * [Autoreg](#autoreg)
    * [Static TF Broadcaster](#static-tf-broadcaster)

## Nodes
### Roboreg Server
```shell
ros2 launch ros2_roboreg server.launch.py -s
```

#### Subscriped Topics
* `image_rect_color`
* `camera_info`: 
* `joint_states`: 
* `point_cloud/cloud_registered`: 

#### Services
* `~/collect_data`: 
* `~/register`: 
* `~/save_synced_data`: 

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
