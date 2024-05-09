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
