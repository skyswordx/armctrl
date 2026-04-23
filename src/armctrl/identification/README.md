# identification

## 这个目录做什么

`identification` 负责机械臂动力学参数辨识前后的工程链路：
安全激励轨迹、关节空间硬件采集、数据集格式、后端抽象和离线工具交接。

## 主要组件

- `models.py`：轨迹点、激励 profile、采样行和 manifest 数据模型。
- `trajectories.py`：三层激励轨迹生成：重力扫描、摩擦扫描、傅里叶多关节轨迹。
- `optimization.py`：有限傅里叶候选轨迹评分与选优。
- `safety.py`：执行前的关节位置、速度、加速度限幅检查。
- `backends.py`：`JointRobotIO` 协议、fake 后端和 ARX5 SDK 关节后端。
- `recorder.py`：写入 `raw_samples.csv`、`planned_trajectory.csv` 和 `manifest.json`。
- `runner.py`：把轨迹发送给后端，并按轨迹时间轴采集关节状态。
- `postprocess.py`：生成 `processed_samples.csv`，计算平滑后速度和加速度。
- `tools.py`：生成 URDFly、Pinocchio、FIGAROH、FloBaRoID 的离线交接说明。

## 实现思路

本模块不实现刚体动力学回归矩阵。
`armctrl` 只保证真实机械臂安全运动并记录可复现数据。
回归矩阵、基参数提取、条件数优化、OLS/WLS 求解交给专门工具。

轨迹按风险递增：

1. `gravity_sweep`：慢速单关节扫描，优先用于重力项和 payload 影响。
2. `friction_sweep`：正反向慢速/中速扫描，用于库伦摩擦和粘性摩擦。
3. `fourier_multisine`：有限傅里叶多关节轨迹，用五次包络让起止速度和加速度为零。

第三层支持 `--optimize`。
当前优化方式是多候选随机种子搜索，并用代理特征矩阵条件数评分。
代理特征包含 `q`、`dq`、`ddq`、`sin(q)`、`cos(q)`、`dq²`、`q*dq`、`q*ddq`。
它的目标是先剔除明显差的候选轨迹，不等同于真实动力学回归矩阵 `Y(q,dq,ddq)`。
后续接入 Pinocchio 或 URDFly 后，应把评分函数替换为真实回归矩阵条件数或 Fisher 信息指标。

未来接 Piper 时，应新增一个实现 `JointRobotIO` 的后端，而不是改轨迹、记录和后处理代码。
