- data.txt包含了图像和radar的原始文件名

- 1.txt---13.txt为需要融合的13次扫描，其中第一个是时间戳上最晚，也是一个关键帧，13.txt时间最早，从1.txt到13.txt时间上越来越早。数据以矩阵(125, 18)的形式写入。

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

