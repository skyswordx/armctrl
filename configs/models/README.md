# armctrl models

本目录用于保存项目侧自定义机器人模型，不直接修改 `.venv` 或 SDK wheel 内的模型文件。

## 文件说明

- `X5_camera.urdf`：从当前运行时 `arx5_interface` 的 `X5.urdf` 复制而来，已写入一版“D435i 位于末端正上方”的起步 payload 近似参数。
- `reference/realsense2_description/`：从 Intel RealSense `realsense-ros` 的 `ros2-master` 分支下载的 D435i 参考描述文件。

当前 `X5_camera.urdf` 的 `eef_link` 已写入一组起步参数：

- 质量：`0.072 kg`
- 质心：`[0, 0, 0.040] m`
- 惯量：按 D435i 本体长方体近似

这组参数只代表相机本体，不含支架、螺丝和线缆。
它的目标是先让重力补偿和动力学模型有一个比 `0.0` 更接近真实的起点。

## 运行时选择规则

`Arx5SDKAdapter` 创建真实 SDK 控制器前会按下面顺序选择 URDF：

1. 如果 CLI 或调用方显式传入 `--urdf-path`，使用该路径。
2. 如果 `--model X5` 且没有显式传 `--urdf-path`，优先使用本目录的 `X5_camera.urdf`。
3. 其他情况继续使用 SDK 自带的模型路径。

因此当前带相机的 X5 默认启动方式是：

```bash
uv run arx5ctl health --adapter sdk --model X5 --gravity-compensation --json
```

如果需要临时对比 SDK 原始模型，可显式传入 SDK wheel 内的原始 `X5.urdf`：

```bash
uv run arx5ctl health --adapter sdk --model X5 --urdf-path .venv/lib/python3.12/site-packages/arx5_interface/models/X5.urdf --json
```

## D435i 参考参数

官方 RealSense xacro 中的 D435 模型给出了以下近似尺寸和质量：

- 尺寸：`0.090 m × 0.025 m × 0.02505 m`
- 质量：`0.072 kg`
- 注意：官方 xacro 在惯量段明确提示这些惯量值不可靠，不应直接用于动力学建模。

Intel / RealSense 产品页给出的 D435i 外形尺寸也是 `90 mm × 25 mm × 25 mm`。
不同 datasheet 版本里的整机质量可能有差异，最终应优先使用实测值。

## 等效质量、质心和惯量计算

URDF（统一机器人描述格式）的 `<inertial>` 写法要求：

- `<origin xyz="...">` 填该 link 坐标系下的质心位置；
- `<mass value="...">` 填总质量；
- `<inertia ...>` 填绕质心坐标系的惯量矩阵，单位是 `kg·m²`。

如果只把 D435i 当成一个长方体 payload，并把它等效到 `eef_link`，可按下面步骤计算。

设 payload 质量为 `m`，长方体在 `eef_link` 坐标系下三个方向尺寸为 `a, b, c`。
若相机在夹爪正上方，且相机质心相对 `eef_link` 原点的位置是：

```text
r = [x_cam, y_cam, z_cam]
```

则 URDF 中可先写：

```xml
<inertial>
  <origin xyz="x_cam y_cam z_cam" rpy="0 0 0"/>
  <mass value="m"/>
  <inertia ixx="Ixx" ixy="0" ixz="0" iyy="Iyy" iyz="0" izz="Izz"/>
</inertial>
```

长方体绕自身质心、且坐标轴与 `eef_link` 对齐时：

```text
Ixx = 1/12 * m * (b² + c²)
Iyy = 1/12 * m * (a² + c²)
Izz = 1/12 * m * (a² + b²)
```

例如假设：

- `m = 0.072 kg`
- 相机长边沿 `eef_link` 的 `y` 方向：`a=0.025 m, b=0.090 m, c=0.025 m`
- 相机贴在夹爪上方，无支架高度，质心近似 `r=[0, 0, 0.0125]`

则：

```text
Ixx ≈ 5.235e-5 kg·m²
Iyy ≈ 7.500e-6 kg·m²
Izz ≈ 5.235e-5 kg·m²
```

如果要把相机、支架、线缆合并成一个等效 payload：

```text
M = Σ mi
c = (Σ mi * ci) / M
I_total = Σ (Ii + mi * ((||ci-c||²)E - (ci-c)(ci-c)^T))
```

其中：

- `mi` 是第 `i` 个零件质量；
- `ci` 是第 `i` 个零件质心；
- `Ii` 是第 `i` 个零件绕自身质心的惯量；
- `E` 是 `3×3` 单位矩阵。

如果只为了修正重力补偿，一开始可以先测总质量和质心，把惯量近似成小长方体惯量。
惯量对静态重力补偿影响通常小于质量和质心位置。

## 下一步要实测的量

如果要把补偿做准，至少补下面 6 项：

- 总质量 `M`：相机本体 + 支架 + 螺丝 + 线缆有效拖拽质量。
- 质心 `cx`：相对 `eef_link` 原点的前后偏置。
- 质心 `cy`：相对 `eef_link` 原点的左右偏置。
- 质心 `cz`：相对 `eef_link` 原点的上下偏置。
- 安装朝向：相机长边、厚度、光轴分别对齐 `eef_link` 的哪个轴。
- 支架和线缆影响：如果支架偏重或线缆拖拽明显，应并入等效 payload。

建议的起步测量方法：

- 用电子秤称整套末端附加件总质量。
- 用卡尺量 `eef_link` 原点到相机几何中心的前后、左右、上下距离。
- 如果支架形状简单，可把相机和支架分别建成长方体，各自算惯量后再合并。
- 如果后面仍有明显重补误差，再去补测惯量；第一优先级始终是总质量和质心。
