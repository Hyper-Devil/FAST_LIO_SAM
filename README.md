# FAST_LIO_SAM

## Front_end : fastlio2      Back_end : lio_sam

<p align='center'>
    <img src="./FAST_LIO_SAM/pic/cover2.png " alt="drawing" width="200" height ="200"/>
    <img src="./FAST_LIO_SAM/pic/cover4.png" alt="drawing" width="200" height =200/>
    <img src="./FAST_LIO_SAM/pic/cover3.png" alt="drawing" width="200" height =200/>
    <img src="./FAST_LIO_SAM/pic/cover1.png" alt="drawing" width="200" height =200/>
</p>

## Videos : FAST-LIO-SAM [Bilibili_link](https://www.bilibili.com/video/BV12Y4y1g7xN/?vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

## Related worked 

1.[FAST-LIO2](https://github.com/hku-mars/FAST_LIO)为紧耦合的lio slam系统，因其缺乏前端，所以缺少全局一致性，参考lio_sam的后端部分，接入GTSAM进行后端优化。

2.[FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)的作者kim在FAST-LIO2的基础上，添加SC-PGO模块，通过加入ScanContext全局描述子，进行回环修正,SC-PGO模块与FAST-LIO2解耦，非常方便，很优秀的工作。

3.[FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC)的作者yanliang-wang,在FAST_LIO_SLAM的基础上添加了：1.基于Radius Search 基于欧式距离的回环检测搜索，增加回环搜索的鲁棒性；2.回环检测的优化结果，更新到FAST-LIO2的当前帧位姿中，幷进行ikdtree的重构，进而更新submap。

## Contributions  

[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM)的主要贡献：

1.对比[FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM/tree/bf975560741c425f71811c864af5d35aa880c797) 与 [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC) 使用外部接入的PGO回环检测模块进行后端优化 ，FAST_LIO_SAM 将LIO-SAM的后端GTSAM优化部分移植到FAST-LIO2的代码中，数据传输处理环节更加清晰。

2.增加关键帧的保存，可通过rosservice的指令对地图和轨迹进行保存。

3.FAST_LIO_SLAM中的后端优化，只使用了GPS的高层进行约束，GPS的高层一般噪声比较大，所以添加GPS的XYZ三维的postion进行GPS先验因子约束。

## Prerequisites

- Ubuntu 18.04 and ROS Melodic
- PCL >= 1.8 (default for Ubuntu 18.04)
- Eigen >= 3.3.4 (default for Ubuntu 18.04)
- GTSAM >= 4.0.0(tested on 4.0.0-alpha2)

## Build

```shell
cd YOUR_WORKSPACE/src
git clone https://github.com/kahowang/FAST_LIO_SAM.git
cd ..
catkin_make
```

## 2026.04.03 定向改动

1. 里程计 `/Odometry` 补充 `twist` 输出，并发布 `pose/twist` 协方差。
2. 新增累积点云发布话题 `/accumulated_map_points`：在 `camera_init` 下累积并降采样，再转换到 `body` 坐标系发布。
3. 累积点云发布加入降采样缓存回写，避免历史点无限增长。
4. 在预处理与激光回调链路增加 NaN 过滤，避免无效点进入特征提取与建图。
5. GNSS 融合新增“源头对齐”：可使用 IMU(ENU) 姿态在启动阶段自动完成 `GNSS local -> SLAM world(first frame)` 旋转与平移初始化。
6. GNSS 约束新增 YAML 可调参数，支持时间对齐窗口、协方差门控、方差下限、速度异常剔除，并提供 fallback 参数（仅在非标准数据源时使用）。

## Quick test

### Loop clousre：

#### 1 .For indoor dataset 

Videos : [FAST-LIO-SAM' videos](https://www.bilibili.com/video/BV12Y4y1g7xN?spm_id_from=444.41.list.card_archive.click&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

dataset is from yanliang-wang 's [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC)  ,[dataset](https://drive.google.com/file/d/1NGTN3aULoTMp3raF75LwMu-OUtzUx-zX/view?usp=sharing) which includes `/velodyne_points`(10Hz) and `/imu/data`(400Hz).

```shell
roslaunch fast_lio_sam mapping_velodyne16.launch
rosbag play  T3F2-2021-08-02-15-00-12.bag  
```

<p align ="center">
<img src = "./FAST_LIO_SAM/pic/indoor.gif "  alt ="car" width = 60%  height =60%; "/>
</p>


#### 2 .For outdoor dataset

dataset is from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) **Walking dataset:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

Videos : [FAST-LIO-SAM' videos](https://www.bilibili.com/video/BV12Y4y1g7xN?spm_id_from=444.41.list.card_archive.click&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

```shell
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_dataset.launch
rosbag  play  walking_dataset.bag
```

<div align="left">
<img src = "./FAST_LIO_SAM/pic/outdoor_1.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
<img src = "./FAST_LIO_SAM/pic/outdoor_2.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
</div>

#### 3.save_map

输入如下指令到terminal中，地图文件将会保存在应文件夹中

```shell
rosservice call /save_map "resolution: 0.0
destination: ''" 
success: True
```

#### 4.save_poes

输入如下指令到terminal中，poes文件将会保存在相应文件夹中

```shell
rosservice call /save_pose "resolution: 0.0
destination: ''" 
success: False
```

evo 绘制轨迹

```shell
evo_traj kitti optimized_pose.txt without_optimized_pose.txt -p
```

| ![evo1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo1.png) | ![evo2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo2.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

#### 5.some config 

```shell
# Loop closure
loopClosureEnableFlag: true		      # use loopclousre or not 
loopClosureFrequency: 4.0                     # Hz, regulate loop closure constraint add frequency
surroundingKeyframeSize: 50                   # submap size (when loop closure enabled)
historyKeyframeSearchRadius: 1.5             # meters, key frame that is within n meters from current pose will be considerd for loop closure
historyKeyframeSearchTimeDiff: 30.0           # seconds, key frame that is n seconds older will be considered for loop closure
historyKeyframeSearchNum: 20                  # number of hostory key frames will be fused into a submap for loop closure
historyKeyframeFitnessScore: 0.3              # icp threshold, the smaller the better alignment

# visual iktree_map  
visulize_IkdtreeMap: true

# visual iktree_map  
recontructKdTree: true

savePCDDirectory: "/fast_lio_sam_ws/src/FAST_LIO_SAM/PCD/"        # in your home folder, starts and ends with "/". Warning: the code deletes "LOAM" folder then recreates it. See "mapOptimization" for implementation
```



### Use GPS：

#### 1.dataset

dataset is from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) **Park dataset:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

Videos : [FAST-LIO-SAM' videos](https://www.bilibili.com/video/BV12Y4y1g7xN?spm_id_from=444.41.list.card_archive.click&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

```shell
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_parking_dataset.launch
rosbag  play  parking_dataset.bag
```

Line Color define:  path_no_optimized(blue)、path_updated(red)、path_gnss(green)

<div align="left">
<img src = "./FAST_LIO_SAM/pic/gps_optimized_path.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
<img src = "./FAST_LIO_SAM/pic/gps_optimized_with_map.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
</div>

#### 2.save_map

输入如下指令到terminal中，地图文件将会保存在应文件夹中

```shell
rosservice call /save_map "resolution: 0.0
destination: ''" 
success: True
```

FAST-LIO  Map (no gnss prior factor)  Red   ;    FAST-LIO-SAM  (with gnss prior factor) Blue

<p align ="center">
<img src = "./FAST_LIO_SAM/pic/gps_map.gif "  alt ="car" width = 60%  height =60%; "/>
</p>

#### 3.save_poes

输入如下指令到terminal中，poes文件将会保存在相应文件夹中

```
rosservice call /save_pose "resolution: 0.0
destination: ''" 
success: False
```

evo 绘制轨迹

```
evo_traj kitti gnss_pose.txt optimized_pose.txt  -p
```

| FAST-LIO  (no gnss prior factor)                             | FAST-LIO-SAM  (with gnss prior factor)                       |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![evo_no_optimized](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo_no_optimized.png) | ![evo_optimized](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo_optimized.png) |



#### 4.some config 

```shell
# GPS Settings
useImuHeadingInitialization: true            # 推荐开启：用 IMU(ENU) 对齐 GNSS local 与 SLAM world
useGpsElevation: true                        # RTK 高程可用时建议 true
gpsCovThreshold: 0.5                         # RTK 场景推荐 0.2~1.0
poseCovThreshold: 0                          # 0 表示不按位姿协方差抑制 GPS 因子

# GNSS robust settings (RTK + ENU)
gnssCoordinateSystem: ENU                    # NavSatFix + GeographicLib 推荐 ENU
gnssYawOffsetDeg: 0.0                        # 仅在 useImuHeadingInitialization=false 时作为 fallback
gnssInvertX: false                           # 仅在 useImuHeadingInitialization=false 时作为 fallback
gnssInvertY: false                           # 仅在 useImuHeadingInitialization=false 时作为 fallback
gnssTimeAlignWindow: 0.10                    # LiDAR-GNSS 匹配窗口，推荐 0.03~0.15
gnssMinVarianceXY: 0.01                      # XY 因子方差下限，推荐 0.005~0.05
gnssMinVarianceZ: 0.04                       # Z 因子方差下限，推荐 0.02~0.20
gnssFactorMinDistance: 1.0                   # GPS 因子最小间距，推荐 0.5~2.0m
gnssMinFixStatus: 0                          # 最低 Fix 状态，0=STATUS_FIX
gnssRejectUnknownCovariance: true            # 拒绝未知协方差帧
gnssEnableVelocityCheck: true                # 速度异常剔除
gnssMaxValidVelocity: 60.0                   # 速度上限，根据车速上限设置
```

#### 5.GNSS 参数对精度影响与调参方法（helios.yaml）

1. 坐标系与首帧对齐（最关键）

- `useImuHeadingInitialization`：影响最大。开启后，用 IMU(ENU) 姿态把 GNSS 本地坐标系对齐到 SLAM 首帧世界系，解决“GNSS 北向地图 vs SLAM 首帧地图”不一致问题。
- `gnssCoordinateSystem`：必须与上游定义一致。`NavSatFix + GeographicLib` 通常应为 `ENU`。配置错误会导致整体旋转偏差。

2. 外参误差（系统性偏差）

- `mapping/extrinT_Gnss2Lidar`：GNSS 天线到 LiDAR 的平移外参（杆臂）。误差会导致轨迹平移偏差与转弯时附加误差。
- `mapping/extrinR_Gnss2Lidar`：旋转外参。误差会把 GNSS 约束投到错误方向，导致优化轨迹持续被拉偏。

3. 质量门控（稳定性）

- `gpsCovThreshold`：GPS 方差门限。过小会“吃不到”GPS 因子，过大会把低质量点引入图优化。
- `gnssMinFixStatus` 与 `gnssRejectUnknownCovariance`：保证只接入可靠 GNSS 解。
- `gnssEnableVelocityCheck` / `gnssMaxValidVelocity`：剔除跳点，避免瞬时错误把图拉坏。

4. 因子权重与频率（精度/鲁棒折中）

- `gnssMinVarianceXY` / `gnssMinVarianceZ`：方差下限。太小会过度相信 GNSS，太大会削弱 GNSS 作用。
- `gnssFactorMinDistance`：GPS 因子间距。太小计算开销大、噪声注入多；太大约束稀疏。
- `gnssTimeAlignWindow`：LiDAR-GNSS 最近邻时间窗口。太小会错过匹配，太大可能配错帧。
- `poseCovThreshold`：控制在位姿退化前后是否注入 GPS，`0` 表示不通过位姿协方差抑制。

5. 建议调参顺序（RTK + 九轴 ENU）

1) 先固定坐标系链路：`useImuHeadingInitialization=true`，`gnssCoordinateSystem=ENU`，外参先用标定值。
2) 再做质量门控：把 `gpsCovThreshold` 调到 0.3~0.8，开启未知协方差剔除与速度检查。
3) 再调权重：`gnssMinVarianceXY` 从 0.02 往 0.005 逐步减小，观察是否出现“被 GNSS 拉扯”。
4) 最后调密度：`gnssFactorMinDistance` 在 0.5~2.0m 间按场景选择，车速高可适当增大。

> 说明：`gnssYawOffsetDeg`、`gnssInvertX`、`gnssInvertY` 是 fallback 选项，仅建议在上游坐标语义不标准、且 `useImuHeadingInitialization=false` 时使用。

#### 6.some fun

when you want to see the path in the Map [satellite map](http://dict.youdao.com/w/satellite map/#keyfrom=E2Ctranslation)，you can also use [Mapviz](http://wiki.ros.org/mapviz)p  plugin . You can refer to  my [blog](https://blog.csdn.net/weixin_41281151/article/details/120630786?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165569598716782246421813%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165569598716782246421813&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-120630786-null-null.142^v17^pc_search_result_control_group,157^v15^new_3&utm_term=MAPVIZ&spm=1018.2226.3001.4187) on CSDN.

<div align="left">
<img src = "./FAST_LIO_SAM/pic/mapviz_1.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
<img src = "./FAST_LIO_SAM/pic/mapviz_2.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
</div>



## Attention:

1.FAST-LIO2中对pose姿态是使用so3表示，而gtsam中，输入的relative_pose姿态是Euler RPY形式表示，需要使用罗德里格斯的公式进行转换更新。

2.参考yanliang-wang [FAST-LIO-LC](https://github.com/yanliang-wang/FAST_LIO_LC)中的iktree  reconstruct 

3.在walking数据集中，因为有个别数据是在同一个地方不断手持旋转激光雷达，旋转激光雷达的角度达到了保存关键帧的阈值，在短时间内，保存了多帧相似的关键帧，导致ISAM2出现特征退化，进而里程计跑飞，可以根据数据集的情况适当调整关键帧选取的阈值参数。

4.添加GPS prior 先验因子的部分diamante，参考lio_sam的先验因子部分，对比于kim的FAST-LIO-SLAM，FAST-LIO-SLAM中只是用了GPS的高层约束，并没有使用xy方向的约束，而GPS在高层(Z轴)的误差比较大，优化过程中容易引入误差。

5.GPS先验因子中，**"useGpsElevation"**是否选择GPS的高层约束，默认不使用，因为GPS的高层噪声比较大。

6.LIO-SAM 中使用**ekf_localization_node**这个ROS Package 把GPS的WGS84 坐标系 转到 World系下，FAST-LIO-SAM考虑到尽量与外部的ROS package 解耦，调用 **GeographicLib**进行坐标转换。



## some problems:

1.历史问题：GNSS经纬高噪声协方差与坐标系不一致，曾导致图优化被异常拉扯。

2.当前改进：默认使用 `useImuHeadingInitialization=true` + `gnssCoordinateSystem=ENU`，在首帧对齐 `GNSS local -> SLAM world`，并在同一链路内完成协方差变换与质量门控。



## UpdateLogs:

根据网友的运行和提示，进行了代码的一些bug更新与修改，更新日志如下，欢迎大家多提issues，感谢大家~

https://github.com/kahowang/FAST_LIO_SAM/blob/master/%E6%9B%B4%E6%96%B0%E6%97%A5%E5%BF%97.md



## Cite the Work 
If you use this repository in your academic research, a BibTeX citation is appreciated: 
```
@misc{wang2022fast_lio_sam,
  title={FAST-LIO-SAM: FAST-LIO with Smoothing and Mapping.},
  author={Wang, Jiahao},
  howpublished={\url{https://github.com/kahowang/FAST_LIO_SAM}},
  year={2022}
}
```
or, you can add a footnote link of this repository: 
`https://github.com/kahowang/FAST_LIO_SAM` 



## Acknowledgements 

​	In this project, the LIO module refers to [FAST-LIO](https://github.com/hku-mars/FAST_LIO) and the pose graph optimization refers to [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM) and [LIO_SAM](https://github.com/TixiaoShan/LIO-SAM).The mainly idea is for [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC).Thanks there great work .

​	Also thanks yanliang-wang、minzhao-zhu、peili-ma  's  great help .

​																																																																	edited by kaho 2022.6.20
