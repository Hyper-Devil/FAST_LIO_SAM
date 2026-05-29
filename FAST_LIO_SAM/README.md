## Local Modifications Log (Hyper-Devil)

This fork contains the following modifications on top of the upstream FAST_LIO_SAM. Changes are listed in reverse-chronological order.

---

### 2026-05-16 — Runtime health alert topic

Added runtime health alerts from `fastlio_sam_mapping` on `/fast_lio_sam/runtime_health_alert` using `diagnostic_msgs/DiagnosticArray`.

Downstream consumers only need to parse the highest `DiagnosticStatus.level`:

| Level | Meaning | Downstream action |
|---|---|---|
| `0` (`OK`) | Recovered after a previous warning/error. | Clear the active SLAM health alarm. |
| `1` (`WARN`) | Degraded scan-to-map alignment or pose stability risk. | Mark SLAM as risky but still publishing. |
| `2` (`ERROR`) | Tracking lost, severe pose jump/flicker/rollback, or strong map-overlap risk. | Treat SLAM output as unhealthy. |

Notes:

1. `/Odometry` timeout/断流 is intentionally monitored downstream, not by this node.
2. Diagnostic `values` are for debugging and replay analysis only; production logic should rely on `level`.
3. Thresholds live under `runtime_health` in every `config/*.yaml` as a safe fallback; the currently used RS LiDAR values were calibrated from `config/helios_bistu.yaml` and copied to the other configs. They can be recalibrated from a known-good bag by temporarily setting `runtime_health/metrics_log_path: "/tmp/fast_lio_sam_runtime_health.csv"` in the active YAML, then replaying:

```bash
roslaunch fast_lio_sam mapping_rs_bistu.launch use_sim_time:=true
rosbag play --clock /media/whd/ITGZ_NOFAN/USED_ROSBAG_2512/2025-08-30-16-00-21.bag
python3 scripts/calibrate_runtime_health.py /tmp/fast_lio_sam_runtime_health.csv
```

---

### 2026-05-14 — `/accumulated_map_points` async 5Hz near-field density

**Goal:** Provide a denser vehicle-local accumulated cloud without reintroducing the main-thread lag previously seen on `/accumulated_map_points`. The effective local horizontal area is now **20 m × 20 m = 400 m²** by default, instead of the older ±20 m CropBox footprint (40 m × 40 m = 1600 m²).

**Fixes applied in `src/laserMapping.cpp` and `config/helios_bistu.yaml`:**

1. **Async accumulated-map worker** — the mapping loop now only snapshots the current undistorted scan and pose into a bounded queue; crop, accumulation, voxel filtering, body-frame conversion, and ROS publishing run in a background thread.

2. **5 Hz publish rate** — `/accumulated_map_points` is throttled by `publish/accum_map_pub_hz` (default `5.0`) instead of running every LiDAR frame. If no one subscribes, the worker clears its cache and does no heavy processing.

3. **Forward-dense / rear-coarse local map** — points are cropped in the latest body frame using configurable ranges (`10 m` forward, `10 m` backward, `±10 m` side, `±3 m` z by default). Forward points (`x >= 0`) use `0.1 m` voxel leaf size; rear points use `0.3 m`.

4. **Point-count degradation** — if the published cloud exceeds `publish/accum_map_max_points` (default `250000`), the worker falls back to a whole-local-cloud `0.3 m` voxel filter and emits a throttled warning.

Build validation: `source /opt/ros/noetic/setup.bash && cd /home/whd/catkin_slam && catkin_make` completed successfully and rebuilt `fastlio_sam_mapping`.

---

### 2026-04-16 — `/accumulated_map_points` performance fix (voxel + local crop)

**Problem:** The `/accumulated_map_points` topic ran an unbounded VoxelGrid downsampling (0.25 m leaf) and full inverse-transform every LiDAR frame in the main thread. After ~200 s of driving the accumulated cloud grew to ~700 k points, causing the main loop to fall >1 s behind and producing a visible TF lag on `camera_init → body`.

**Root cause confirmed** via `scripts/diag_timestamp_lag.py`: IMU and LiDAR timestamps were stable throughout the 300 s bag; `/Odometry` lag only exploded at ~200 s, tracking the growth of the accumulated cloud exactly.

**Fixes applied in `src/laserMapping.cpp`:**

1. **`/Laser_map` subscriber guard** — `ikdtree.flatten()` and `publish_map()` are now skipped unless someone is actively subscribed, eliminating a per-frame O(N) tree copy when the topic is unused.

2. **`/accumulated_map_points` — coarser voxel + local crop:**
   - Leaf size increased from `0.25 m` → `1.0 m` (≈ 64× fewer points in the VoxelGrid pass).
   - After downsampling, a `pcl::CropBox` retains only points within **±20 m** of the current body position (world frame). The cropped result is written back to `pcl_wait_pub`, so the cache size is now bounded by local point density rather than total travel distance.
   - Processing time is O(constant) regardless of bag length; `/Odometry` lag stays flat at ~30 ms throughout the 300 s bag.

Added `scripts/diag_timestamp_lag.py`: subscribes to `/imu/data`, `/rslidar_points`, `/Odometry`, `/Laser_map`, `/accumulated_map_points` and logs `(topic, sim_time, lag, cloud_size)` to `~/catkin_slam/timestamp_lag.csv` for post-run analysis.

---

### 2026-04-10 — Dependency upgrades and MVS conflict fix

1. Livox dependency upgraded from **livox_ros_driver** to **livox_ros_driver2** in `CMakeLists.txt`, `package.xml`, and source includes.
2. GeographicLib dependency validated on Ubuntu 20.04 (`libgeographic-dev`).
3. GTSAM dependency installation validated with BorgLab PPA (`libgtsam-dev`, `libgtsam-unstable-dev`).
4. Added CMake-level system `libusb` selection to avoid `/opt/MVS` linker interference — scope is this project only, no global environment change.

Updated files: `CMakeLists.txt`, `package.xml`, `src/preprocess.h`, `src/preprocess.cpp`, `src/laserMapping.cpp`.

---

### 2026-04-09 — `use_sim_time` support

Added `<param name="use_sim_time" value="true"/>` to `launch/mapping_rs.launch` for correct behaviour when replaying bags with `rosbag play --clock`.

---

### 2026-04-03 — RS LiDAR support, accumulated map, GNSS tuning, performance fix

| Commit | Change |
|---|---|
| RS data format fix | Added `config/helios.yaml` (Robosense Helios 32-line config); fixed point-field parsing in `src/preprocess.cpp` / `src/preprocess.h`; updated RViz config. Added `scripts/imu_gps_pose_tf_node.py` to republish IMU+GPS as a combined TF. |
| Accumulated map + twist | Added `/accumulated_map_points` publisher (world-accumulated, body-frame output). Added velocity (twist) output to `/Odometry`. |
| GNSS tuning | Improved GNSS-LIO heading initialisation from ENU IMU orientation; tuned covariance gates (`gpsCovThreshold`, `poseCovThreshold`) for higher accuracy without GNSS dependency. Updated `config/helios.yaml` and `launch/mapping_rs.launch`. |
| Performance fix | Resolved CPU bottleneck from unbounded `pcl_wait_save` growth and excessive logging output. Cleaned up stale log files. |

## Related Works

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
2. [IKFOM](https://github.com/hku-mars/IKFoM): A Toolbox for fast and high-precision on-manifold Kalman filter.
3. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): One of the implementation of FAST-LIO in robot's planning.
4. [R2LIVE](https://github.com/hku-mars/r2live): A high-precision LiDAR-inertial-Vision fusion work using FAST-LIO as LiDAR-inertial front-end.
5. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): Model Predictive Control for Trajectory Tracking on Differentiable Manifolds.
6. [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): The integration of FAST-LIO with [Scan-Context](https://github.com/irapkaist/scancontext) **loop closure** module.
7. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): The integration of FAST-LIO with **Re-localization** function module.

## FAST-LIO
**FAST-LIO** (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. Our package address many key issues:
1. Fast iterated Kalman filter for odometry optimization;
2. Automaticaly initialized at most steady environments;
3. Parallel KD-Tree Search to decrease the computation;

## FAST-LIO 2.0 (2021-07-05 Update)
<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->

<div align="left">
<img src="doc/real_experiment2.gif" width=49.6% />
<img src="doc/ulhkwh_fastlio.gif" width = 49.6% >
</div>

**Related video:**  [FAST-LIO2](https://youtu.be/2OvjGnxszf8),  [FAST-LIO1](https://youtu.be/iYCY6T79oNU),  [FAST-LIO2 + Scan-context Loop Closure](https://www.youtube.com/watch?v=nu8j4yaBMnw)

**Pipeline:**
<div align="center">
<img src="doc/overview_fastlio2.svg" width=99% />
</div>

**New Features:**
1. Incremental mapping using [ikd-Tree](https://github.com/hku-mars/ikd-Tree), achieve faster speed and over 100Hz LiDAR rate.
2. Direct odometry (scan to map) on Raw LiDAR points (feature extraction can be disabled), achieving better accuracy.
3. Since no requirements for feature extraction, FAST-LIO2 can support many types of LiDAR including spinning (Velodyne, Ouster) and solid-state (Livox Avia, Horizon, MID-70) LiDARs, and can be easily extended to support more LiDARs.
4. Support external IMU.
5. Support ARM-based platforms including Khadas VIM3, Nivida TX2, Raspberry Pi 4B(8G RAM).

**Related papers**: 

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

**Contributors**

[Wei Xu 徐威](https://github.com/XW-HKU)，[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)，[Dongjiao He 贺东娇](https://github.com/Joanna-HE)，[Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)，[Jiarong Lin 林家荣](https://github.com/ziv-lin)，[Zheng Liu 刘政](https://github.com/Zale-Liu), [Borong Yuan](https://github.com/borongyuan)

<!-- <div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div> -->

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.3. **livox_ros_driver2**
Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2).

*Remarks:*
- Since the FAST-LIO must support Livox serials LiDAR firstly, the **livox_ros_driver2** must be installed and **sourced** before running any FAST-LIO launch file.
- How to source? Add `source $LIVOX_ROS_DRIVER2_WS/devel/setup.bash` to `~/.bashrc`, where `$LIVOX_ROS_DRIVER2_WS` is the workspace directory of livox_ros_driver2.

### 1.4. **GeographicLib**

FAST_LIO_SAM uses GeographicLib in GNSS related modules.

Install on Ubuntu 20.04:

```bash
sudo apt-get update
sudo apt-get install -y libgeographic-dev
```

Quick verification:

```bash
dpkg -s libgeographic-dev | grep -E "Status|Version"
```

If CMake still cannot find GeographicLib, set one of the following:

```bash
export CMAKE_PREFIX_PATH=/usr:$CMAKE_PREFIX_PATH
# or pass GeographicLib_DIR when building
```

### 1.5. **GTSAM**

Install (Ubuntu 20.04):

```bash
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

Quick verification:

```bash
dpkg -s libgtsam-dev | grep -E "Status|Version"
```

### 1.6. **/opt/MVS libusb conflict note**

If your environment contains `/opt/MVS` in `LD_LIBRARY_PATH`, this project now forces system `libusb` selection in CMake for `fast_lio_sam` build/link.

- Scope: only this project CMake, no global environment change.
- Goal: avoid linker/runtime mismatch between PCL IO and non-system `libusb`.
- This change does **not** modify or uninstall MVS.


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver2 before build (follow 1.3 **livox_ros_driver2**)
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```
## 3. Directly run
Noted:
A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.
B. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important for the forward propagation and backwark propagation.
### 3.1 For Avia
Connect to your PC to Livox Avia LiDAR by following [Livox-ros-driver2 installation](https://github.com/Livox-SDK/livox_ros_driver2), then
```
    cd ~/$FAST_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch fast_lio mapping_avia.launch
    # use your installed livox_ros_driver2 launch file here
    # e.g. roslaunch livox_ros_driver2 <your_launch>.launch
```
- For livox serials, FAST-LIO requires per-point timestamps from `livox_ros_driver2/CustomMsg` for motion undistortion.
- Please configure publish rate and launch settings in your livox_ros_driver2 launch configuration before building/running.

### 3.2 For Livox serials with external IMU

mapping_avia.launch theratically supports mid-70, mid-40 or other livox serial LiDAR, but need to setup some parameters befor run:

Edit ``` config/avia.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ```
3. Translational extrinsic: ``` extrinsic_T ```
4. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.
- FAST-LIO produces a very simple software time sync for livox LiDAR, set parameter ```time_sync_en``` to ture to turn on. But turn on **ONLY IF external time synchronization is really not possible**, since the software time sync cannot make sure accuracy.

### 3.3 For Velodyne or Ouster (Velodyne as an example)

Step A: Setup before run

Edit ``` config/velodyne.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ``` (both internal and external, 6-aixes or 9-axies are fine)
3. Line number (we tested 16, 32 and 64 line, but not tested 128 or above): ``` scan_line ```
4. Translational extrinsic: ``` extrinsic_T ```
5. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame).

Step B: Run below
```
    cd ~/$FAST_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch fast_lio mapping_velodyne.launch
```

Step C: Run LiDAR's ros driver or play rosbag.

### 3.4 PCD file save

Set ``` pcd_save_enable ``` in launchfile to ``` 1 ```. All the scans (in global frame) will be accumulated and saved to the file ``` FAST_LIO/PCD/scans.pcd ``` after the FAST-LIO is terminated. ```pcl_viewer scans.pcd``` can visualize the point clouds.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

## 4. Rosbag Example
### 4.1 Livox Avia Rosbag
<div align="left">
<img src="doc/results/HKU_LG_Indoor.png" width=47% />
<img src="doc/results/HKU_MB_002.png" width = 51% >

Files: Can be downloaded from [google drive](https://drive.google.com/drive/folders/1YL5MQVYgAM8oAWUm7e3OGXZBPKkanmY1?usp=sharing)

Run:
```
roslaunch fast_lio mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag

```

### 4.2 Velodyne HDL-32E Rosbag

**NCLT Dataset**: Original bin file can be found [here](http://robots.engin.umich.edu/nclt/).

We produce [Rosbag Files](https://drive.google.com/drive/folders/1VBK5idI1oyW0GC_I_Hxh63aqam3nocNK?usp=sharing) and [a python script](https://drive.google.com/file/d/1leh7DxbHx29DyS1NJkvEfeNJoccxH7XM/view) to generate Rosbag files: ```python3 sensordata_to_rosbag_fastlio.py bin_file_dir bag_name.bag```
    
Run:
```
roslaunch fast_lio mapping_velodyne.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 5.Implementation on UAV
In order to validate the robustness and computational efficiency of FAST-LIO in actual mobile robots, we build a small-scale quadrotor which can carry a Livox Avia LiDAR with 70 degree FoV and a DJI Manifold 2-C onboard computer with a 1.8 GHz Intel i7-8550U CPU and 8 G RAM, as shown in below.

The main structure of this UAV is 3d printed (Aluminum or PLA), the .stl file will be open-sourced in the future.

<div align="center">
    <img src="doc/uav01.jpg" width=40.5% >
    <img src="doc/uav_system.png" width=57% >
</div>

## 6.Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) and [Loam_Livox](https://github.com/hku-mars/loam_livox).
