# 3DRef Seq1：连续玻璃与镜面

完整Seq1已下载、提取并通过结构/时间配对检查，待地图清理评测。[官方项目](https://3dref.github.io/) · [原始归档](https://robotics.shanghaitech.edu.cn/static/datasets/3DRef/seq1.tar.bz2) · [总索引与论文](../../README.md)。本次范围仅Seq1。

| 传感器目录 | 帧数 | 点数 |
| --- | ---: | ---: |
| `pointcloud/hesai/` | 3732 | 128532365 |
| `pointcloud/livox/` | 3732 | 91865754 |
| `pointcloud/ouster/` | 3732 | 321572978 |

共11196帧、541971097点；展开PLY为59,697,522,615字节。原档为 `raw/seq1.tar.bz2`（21,734,882,202字节），内部点云包为 `extracted/raw/seq1/raycast/pointcloud.zip`（20,465,534,960字节）。三份 `<sensor>_pose.txt` 和mesh在 `extracted/raw/seq1/`；RGB仍保留在原档，未展开。

## 读取与标签

PLY为ASCII，字段顺序 `x y z nx ny nz intensity ring return label`；无逐点时间字段，本包没有IMU。文件名为帧级时间，配对TUM格式 `timestamp x y z qx qy qz qw`。点云是雷达局部坐标，按 [上游固定版本](https://github.com/STAR-Center/Reflection_detection/blob/3d4bb102f0c13486fb430f5ad01868b9232d0c0a/src/multiple_frame_3dref.cpp#L110-L142) 使用 `p_global = R(q) * p_lidar + t`。

标签：0未标注、1正常表面、2玻璃、3镜面、4其他反射材质、5虚反射点、6玻璃后的真实表面/障碍。清理应区分虚点5与真实表面2/3/4/6；这些不是动态运动标签。

三份pose各3732行，覆盖375.281558 s；最大源时间间隔2.000459 s。全部文件名时间与pose精确相等，无重复/未配对；位姿来自MA-LIO估计，不是独立轨迹真值。

## 检查记录

全部文件完成ZIP解码/CRC、PLY头/字段、实际行数和落盘字节检查；数值/label/return完整解析限740帧、25949978点，含每种雷达12个时间等分代表帧。Livox样本有3个 `return=0` 点，含义未确认；最稀疏帧只有29点，保留源值并处理密度波动。

- [下载清单](download-manifest.json)：来源、文件与精确字节。
- [逐帧记录](metadata/cloud-files.jsonl) · [结构/数值检查范围](metadata/zip-inspection.json) · [时间与重复检查](metadata/sequence-audit.json)。

数据许可尚未确认；官网页脚许可仅能确认网站许可。
