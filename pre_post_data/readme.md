## 输入、输出数据说明文档

### 融合方式

雷达和图像的融合方式，有3个关键点：

- 多次雷达扫描点云叠加；
- 通过拉长3m，生成点云高度信息；
- 点云通过坐标转换投影到图像中，并合并通道；

### 输入数据

#### **图像：**

```shell
./data/nuscenes/samples/CAM_FRONT/n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402929162460.jpg
```

![](X:\project\CameraRadarFusionNet\data_pre_v2\pre_post_data\n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402929162460.jpg)

#### **雷达**

文件为3_sweep_radar_1.txt---3_sweep_radar_13.txt，为需要融合的13次扫描。其中第一个文件是时间戳上最晚，也是一个关键帧，已经过自我车辆的矫正。数据以矩阵(n_smaple, 18)的形式写入。

```shell
[0]: x
[1]: y
[2]: z
[3]: dyn_prop
[4]: id
[5]: rcs
[6]: vx
[7]: vy
[8]: vx_comp
[9]: vy_comp
[10]: is_quality_valid
[11]: ambig_state
[12]: x_rms
[13]: y_rms
[14]: invalid_state
[15]: pdh0
[16]: vx_rms
[17]: vy_rms
```

```shell
x is front, y is left

vx, vy are the velocities in m/s.
vx_comp, vy_comp are the velocities in m/s compensated by the ego motion.We recommend using the compensated velocities.
invalid_state: state of Cluster validity state.
    (Invalid states)
    0x01	invalid due to low RCS
    0x02	invalid due to near-field artefact
    0x03	invalid far range cluster because not confirmed in near range
    0x05	reserved
    0x06	invalid cluster due to high mirror probability
    0x07	Invalid cluster because outside sensor field of view
    0x0d	reserved
    0x0e	invalid cluster because it is a harmonics
    (Valid states)
    0x00	valid
    0x04	valid cluster with low RCS
    0x08	valid cluster with azimuth correction due to elevation
    0x09	valid cluster with high child probability
    0x0a	valid cluster with high probability of being a 50 deg artefact
    0x0b	valid cluster but no local maximum
    0x0c	valid cluster with high artefact probability
    0x0f	valid cluster with above 95m in near range
    0x10	valid cluster with high multi-target probability
    0x11	valid cluster with suspicious angle
dynProp: Dynamic property of cluster to indicate if is moving or not.
    0: moving
    1: stationary
    2: oncoming
    3: stationary candidate
    4: unknown
    5: crossing stationary
    6: crossing moving
    7: stopped
ambig_state: State of Doppler (radial velocity) ambiguity solution.
    0: invalid
    1: ambiguous
    2: staggered ramp
    3: unambiguous
    4: stationary candidates

pdh0: False alarm probability of cluster (i.e. probability of being an artefact caused by multipath or similar).
    0: invalid
    1: <25%
    2: 50%
    3: 75%
    4: 90%
    5: 99%
    6: 99.9%
    7: <=100%
      

```

3_radar_points.txt为经过筛选后的雷达点云，也是以矩阵的形式表示，其形状为(n_pts, 21)。



#### 融合后的图像

可视化投影到图像坐标系的点云如下：

![](X:\project\CameraRadarFusionNet\data_pre_v2\pre_post_data\radar2image.png)

3_input_data_1.txt---3_input_data_5.txt：融合后的五个通道，分别表示(r,g,b,dis,rcs)。前三个通道是图像原有通道，后两个通道为雷达投影到图像的距离和雷达散射截面通道。数据的形状为(384, 640, 5)

0003_gt.png：可视化后图像（未将雷达信息加入可视化）

![](X:\project\CameraRadarFusionNet\data_pre_v2\pre_post_data\0003_gt.png)

## 输出数据：

3_regression.txt: 预测框的偏置大小，形状为(n_boxes, 4)，其中n_boxes=46035

3_classification.txt：预测框的置信度，形状为(n_boxex, 8)

3_anchors.txt：表示先验框的大小和位置，形状为(n_boxes, 4)

3_boxes.txt：3_regression.txt和3_anchors.txt叠加后真实框的位置和大小，形状为(n_boxes, 4)

3_clipped_boxes.txt：裁剪后的真实框的位置和大小，形状为(<=n_boxes, 4)

3_filtered_boxes.txt: nms之后的真实框的位置和大小，形状为(<=n_boxes, 4)

3_filtered_cla.txt：nms之后的真实框的置信度，形状为(<=n_boxes, 1)

3_filtered_labels.txt nms之后的真实框的标签，形状为(<=n_boxes, 1)

3_prediction.txt：表示预测的boxes, 置信度以及类别

预测的结果可视化如下图所示：

![](X:\project\CameraRadarFusionNet\data_pre_v2\pre_post_data\0003_pred.png)