# Dual-Ackermann Four-State DARE-LQR Controller

面向 ROS 2 `nav_core` 框架的双阿克曼路径跟踪控制器，主要针对 Ranger Mini V3 等前后轮反相转向底盘。控制器采用四状态离散代数黎卡提方程（Discrete Algebraic Riccati Equation, DARE）在线求解反馈增益，并结合路径曲率前馈、短预瞄转向、长预瞄减速、正倒车统一运动坐标系、终点自然减速和低速停车锁存，实现连续路径跟踪与厘米级终点停车。

> 当前 Ranger Mini V3 调试结果：停车位置误差小于 `1 cm`，航向误差小于 `1°`。
>

> [Ranger Mini V3 效果视频](https://www.xiaohongshu.com/explore/6a51df110000000021018133?xsec_token=AB68jAZqpw4yHAFrqr9d0r3AACLUZA5kRXH2MK-Ds1yCQ=&xsec_source=pc_user)

![Ranger Mini V3](./assets/ranger_mini_v3.jpg)

## 目录

- [1. 主要特性](#1-主要特性)
- [2. 控制流程](#2-控制流程)
- [3. 坐标系、参考点与误差定义](#3-坐标系参考点与误差定义)
- [4. 双阿克曼几何模型](#4-双阿克曼几何模型)
- [5. 四状态 DARE-LQR](#5-四状态-dare-lqr)
- [6. 路径曲率与短预瞄转向](#6-路径曲率与短预瞄转向)
- [7. 长预瞄与速度规划](#7-长预瞄与速度规划)
- [8. 路径捕获与重新捕获](#8-路径捕获与重新捕获)
- [9. 终点减速、停车锁存与倒车修正](#9-终点减速停车锁存与倒车修正)
- [10. 曲率与速度指令平滑](#10-曲率与速度指令平滑)
- [11. 参数配置](#11-参数配置)
- [12. 诊断话题](#12-诊断话题)
- [13. 工程集成与编译](#13-工程集成与编译)
- [14. 参数调试建议](#14-参数调试建议)
- [15. 使用边界](#15-使用边界)

## 1. 主要特性

- **四状态在线 DARE-LQR**：每个控制周期根据当前允许速度和前馈转角重新构造离散系统矩阵，并在线求解反馈增益。
- **前馈与反馈解耦**：路径曲率生成中央转角前馈，LQR 只计算相对于前馈工作点的转角修正量。
- **正倒车统一建模**：将正向和倒向路径统一转换到运动坐标系，避免为倒车重新设计一套误差符号和控制律。
- **双层预瞄**：短预瞄负责提前转向和提前回正，长预瞄仅负责弯前速度约束。
- **两遍参考计算**：先估计允许速度，再使用实际允许速度重新计算预瞄距离和路径参考量。
- **路径捕获速度门控**：初始偏差较大或跟踪过程中重新偏离路径时主动降速。
- **统一 `base_link` 终点误差**：路径跟踪、终点减速和停车锁存使用相同的车辆参考点，避免定位传感器外参与控制终点不一致。
- **终点自然停车**：正向终点不叠加额外的终点航向覆盖项，保持路径切线，避免末端突然转向。
- **倒车低速横向收敛**：仅在倒车终点附近附加小幅横向修正，并在进入停车窗口时衰减。
- **双级平滑**：先平滑曲率，再平滑线速度，同时保持底盘所需的转向比值。
- **完整诊断输出**：发布路径误差、终点误差、LQR 增益、前馈量、反馈量和曲率处理结果。

## 2. 控制流程

```text
局部 base_link 控制路径
        ↓
定位传感器位姿通过外参转换为当前 base_link 位姿
        ↓
正、倒车路径统一转换到运动坐标系
        ↓
删除车辆后方无效路径点，并在必要时延长局部参考路径
        ↓
最近线段投影：e_y、e_psi、路径进度 s
        ↓
当前曲率 kappa_now
短预瞄曲率 kappa_preview
长预瞄弯道速度上限 v_anticipation
        ↓
根据路径曲率、跟踪误差、终点距离和路径捕获状态
进行第一遍目标速度计算
        ↓
使用第一遍允许速度重新计算普通前瞻、短预瞄和长预瞄
        ↓
进行第二遍目标速度计算
        ↓
根据 |v| 和 phi_ff 在线构造 A、B，并迭代求解 DARE
        ↓
Delta_phi_LQR = -K [e_y, e_y_dot, e_psi, e_psi_dot]^T
        ↓
phi_cmd = phi_ff + Delta_phi_LQR + Delta_phi_reverse
        ↓
中央转角 → 双阿克曼内轮转角 → cmd_vel.angular.z
        ↓
曲率限速、曲率平滑、速度平滑和终点锁存
        ↓
发布 cmd_vel 与诊断信息
```

控制器的核心关系为：

```text
短预瞄：直接改变曲率前馈和 LQR 线性化工作点
长预瞄：先限制速度，再通过速度间接改变 A、B、K 和预瞄尺度
DARE-LQR：在当前前馈工作点附近抑制横向误差和航向误差
终点控制：沿路径终端切线自然减速，不使用正向末端强制转向覆盖
```

## 3. 坐标系、参考点与误差定义

### 3.1 局部路径坐标系

控制器接收的局部控制路径应位于车辆局部坐标系中，车辆当前 `base_link` 位于：

```math
(x_r,y_r,\psi_r)=(0,0,0)
```

普通路径跟踪使用路径点的几何位置计算线段方向，不依赖每个路径点的四元数。终端存在外部目标时，终点航向使用目标位姿的四元数；否则使用路径末端向前回看一段距离得到的终端切线方向。

默认终端切线回看距离为：

```math
D_{\mathrm{heading}}=0.65\ \mathrm{m}
```

### 3.2 正倒车统一运动坐标系

定义运动方向符号：

```math
\sigma=
\begin{cases}
+1, & \text{正向行驶}\\
-1, & \text{倒向行驶}
\end{cases}
```

局部路径点从 `base_link` 坐标系转换到统一运动坐标系：

```math
x_m=\sigma x_b
```

```math
y_m=\sigma y_b
```

经过该变换后，无论正向还是倒向行驶，车辆在运动坐标系中都等价为沿正 $x$ 方向前进，因此最近线段投影、路径进度、横向误差和航向误差可以使用同一套算法。

倒车外部终点航向同时补偿 $\pi$：

```math
e_{\psi,t}^{m}
=
\mathrm{wrap}
\left(
\psi_t-\psi_b+\pi
\right)
```

### 3.3 最近线段投影

控制器遍历局部路径线段，将运动坐标系原点投影到距离最近的有效线段，得到：

- 横向误差 $e_y$；
- 航向误差 $e_\psi$；
- 从路径起点到投影点的累计路径进度 $s$。

当前实现的符号约定为：

- $e_y>0$：参考路径位于车辆运动方向的左侧；
- $e_y<0$：参考路径位于车辆运动方向的右侧；
- $e_\psi$：最近路径线段航向相对于运动坐标系正 $x$ 轴的夹角。

误差在进入控制器前经过死区处理：

```math
|e_y|<0.0015\ \mathrm{m}
\Rightarrow e_y=0
```

```math
|e_\psi|<0.0020\ \mathrm{rad}
\Rightarrow e_\psi=0
```

### 3.4 `base_link` 参考点与定位外参

里程计位姿可以对应雷达、惯性测量单元或其他定位传感器，而控制目标对应车辆 `base_link`。控制器通过 `robot_base.extrinsics` 将当前定位传感器位姿转换为当前 `base_link` 位姿：

```text
定位传感器位姿
        ↓ 只进行一次外参变换
当前 base_link 位姿
        ↓
与目标 base_link 位姿计算终点误差
```

终点目标被解释为地图坐标系中的期望 `base_link` 位姿。路径跟踪、终点减速和停车锁存必须使用同一车辆参考点，否则定位外参会表现为固定的厘米级停车偏差。

### 3.5 局部路径预处理

控制器还包含以下工程处理：

1. 运动坐标系中仅保留满足 $x\geq-0.08\ \mathrm{m}$ 的路径点，减少车辆后方路径对最近点搜索的干扰；
2. 过滤后不足两个点时恢复原路径；
3. 路径长度小于普通前瞻距离或最小参考长度时，沿终端切线延长路径；
4. 输入路径少于两个点时，以外部跟踪目标为中心构造一条切线备用路径。

## 4. 双阿克曼几何模型

### 4.1 几何变量

- $L$：前后轴距；
- $W$：轮距；
- $\delta_{\mathrm{in}}$：双阿克曼转弯内侧车轮转角；
- $\phi$：等效中央转角；
- $\kappa$：车辆轨迹曲率。

当前 Ranger Mini V3 默认参数为：

```math
L=0.494\ \mathrm{m},\qquad W=0.364\ \mathrm{m}
```

### 4.2 内轮转角与中央转角

内轮转角转换为中央转角：

```math
\phi
=
\mathrm{atan2}
\left(
L\sin|\delta_{\mathrm{in}}|,
L\cos|\delta_{\mathrm{in}}|+W\sin|\delta_{\mathrm{in}}|
\right)
```

最终符号与 $\delta_{\mathrm{in}}$ 保持一致。

中央转角转换为内轮转角：

```math
\delta_{\mathrm{in}}
=
\mathrm{atan2}
\left(
L\sin|\phi|,
L\cos|\phi|-W\sin|\phi|
\right)
```

最终符号与 $\phi$ 保持一致。

### 4.3 中央转角与曲率

当前实现使用以下双阿克曼等效关系：

```math
\kappa
=
\frac{2\tan\phi}{L}
```

因此，路径曲率对应的中央转角为：

```math
\phi
=
\arctan
\left(
\frac{L\kappa}{2}
\right)
```

### 4.4 转向和最小转弯半径约束

首先根据最小转弯半径和安全余量计算允许的内轮转角：

```math
\delta_{\mathrm{radius}}
=
\arctan
\left(
\frac{L/2}{R_{\min}+\Delta R}
\right)
```

实际可用内轮转角为：

```math
\delta_{\mathrm{usable}}
=
\min
\left(
\delta_{\max},
\delta_{\mathrm{radius}}
\right)
```

再由 $\delta_{\mathrm{usable}}$ 换算得到最大中央转角。最大曲率同时受到中央转角和最小转弯半径限制：

```math
\kappa_{\max}
=
\min
\left(
\left|\frac{2\tan\phi_{\max}}{L}\right|,
\frac{1}{R_{\min}}
\right)
```

按照默认参数，初始化后约有：

```text
最大可用内轮转角 ≈ 0.3791 rad
最大中央转角     ≈ 0.2988 rad
最大曲率         ≈ 1.2469 1/m
```

这些数值由源码根据几何参数在线派生，不直接写入 YAML。

### 4.5 `cmd_vel.angular.z` 的生成

中央转角先转换为内轮转角，再计算角速度指令：

```math
\omega_{\mathrm{cmd}}
=
\mathrm{clip}
\left(
\frac{2|v|\tan\delta_{\mathrm{in}}}{L},
-\omega_{\max},
\omega_{\max}
\right)
```

这里使用 $|v|$ 而不是带符号线速度。该接口设计用于 Ranger 底盘或适配器通过 `angular.z / |linear.x|` 还原转向需求，倒车方向由 `linear.x` 的符号单独表达。因此，不应直接将该 `angular.z` 解释为普通单车模型中的 $v\kappa$。

## 5. 四状态 DARE-LQR

### 5.1 状态向量

状态向量定义为：

```math
X_k=
\begin{bmatrix}
e_y &
\dot e_y &
e_\psi &
\dot e_\psi
\end{bmatrix}^{T}
```

其中：

- $e_y$：横向位置误差；
- $\dot e_y$：横向误差变化率；
- $e_\psi$：航向角误差；
- $\dot e_\psi$：航向角误差变化率。

进入 LQR 前，位置和航向误差分别限幅为：

```math
e_y\in[-0.25,0.25]\ \mathrm{m}
```

```math
e_\psi\in[-0.70,0.70]\ \mathrm{rad}
```

### 5.2 控制输入

LQR 控制输入是相对于路径前馈中央转角的反馈修正量：

```math
u_k=\Delta\phi_k
```

它不是完整的中央转角指令。最终中央转角为：

```math
\phi_{\mathrm{cmd}}
=
\phi_{\mathrm{ff}}
+
\Delta\phi_{\mathrm{LQR}}
+
\Delta\phi_{\mathrm{reverse}}
```

其中：

- $\phi_{\mathrm{ff}}$：路径曲率前馈中央转角；
- $\Delta\phi_{\mathrm{LQR}}$：DARE-LQR 反馈修正；
- $\Delta\phi_{\mathrm{reverse}}$：倒车终点低速横向修正。

### 5.3 离散状态空间模型

控制器每周期重新构造：

```math
X_{k+1}
=
A(v,\Delta t)X_k
+
B(v,\phi_{\mathrm{ff}})\Delta\phi_k
```

状态矩阵为：

```math
A=
\begin{bmatrix}
1 & \Delta t & 0 & 0\\
0 & 0 & v & 0\\
0 & 0 & 1 & \Delta t\\
0 & 0 & 0 & 0
\end{bmatrix}
```

输入矩阵为：

```math
B=
\begin{bmatrix}
0\\
0\\
0\\
-\dfrac{2v\cos\phi_{\mathrm{ff}}}{L}
\end{bmatrix}
```

其中：

- $v$：DARE 模型使用的速度大小；
- $\Delta t$：控制周期；
- $\phi_{\mathrm{ff}}$：前馈曲率对应的中央转角；
- $L$：车辆轴距。

### 5.4 模型速度下限

为避免低速时输入矩阵 $B$ 退化，模型速度采用：

```math
v_{\mathrm{model}}
=
\max
\left(
|v_{\mathrm{cmd}}|,
\max(v_{\mathrm{terminal,min}},0.05)
\right)
```

默认：

```math
v_{\mathrm{terminal,min}}=0.01\ \mathrm{m/s}
```

因此实际 DARE 模型最低使用：

```math
v_{\mathrm{model,min}}=0.05\ \mathrm{m/s}
```

该下限只用于构造 $A$、$B$ 和求解 $K$，实际发布速度仍可低于 $0.05\ \mathrm{m/s}$，也可以为零。

### 5.5 误差变化率构造

源码没有直接完全依赖相邻周期差分，而是将运动学估计与有限差分融合。

横向误差变化率的模型估计为：

```math
\dot e_{y,\mathrm{model}}
=
|v|\sin e_\psi
```

航向误差变化率的模型估计为：

```math
\dot e_{\psi,\mathrm{model}}
=
|v|
\left(
\kappa_{\mathrm{ff}}-\kappa_{\mathrm{prev}}
\right)
```

存在历史误差时，同时计算有限差分：

```math
\dot e_{y,\mathrm{meas}}
=
\frac{e_{y,k}-e_{y,k-1}}{\Delta t}
```

```math
\dot e_{\psi,\mathrm{meas}}
=
\frac{\mathrm{wrap}(e_{\psi,k}-e_{\psi,k-1})}{\Delta t}
```

测量变化率的融合比例为：

```text
直线路径：0.12
曲线路径：0.30
```

随后使用一阶低通滤波：

```math
\dot e_k^{f}
=
\alpha_r\dot e_k^{\mathrm{sample}}
+
(1-\alpha_r)\dot e_{k-1}^{f}
```

默认：

```math
\alpha_r=0.25
```

最终变化率限幅为：

```math
\dot e_y\in[-1,1]\ \mathrm{m/s}
```

```math
\dot e_\psi\in[-2,2]\ \mathrm{rad/s}
```

这种构造降低了定位噪声直接差分导致的转向抖动，同时保留了实际误差变化趋势。

### 5.6 代价函数

无限时域离散 LQR 代价函数为：

```math
J
=
\sum_{k=0}^{\infty}
\left(
X_k^{T}QX_k
+
u_k^{T}Ru_k
\right)
```

默认状态权重为：

```math
Q=
\mathrm{diag}
\left(
12.0,
0.30,
10.0,
0.25
\right)
```

控制权重为：

```math
R=4.2
```

其中，$Q$ 只包含四个状态代价，$v$ 和 $\Delta t$ 位于 $A$、$B$ 中，不属于 $Q$。

### 5.7 DARE 迭代

离散代数黎卡提方程为：

```math
P_{\mathrm{next}}
=
Q+A^{T}PA
-A^{T}PB
\left(R+B^{T}PB\right)^{-1}
B^{T}PA
```

由于系统只有一个控制输入：

```math
B\in\mathbb{R}^{4\times1}
```

所以：

```math
S=R+B^{T}PB
```

为标量。

源码中显式计算：

```text
PB   = P B
S    = R + B^T P B
PA   = P A
AtPB = A^T P B
BtPA = B^T P A
AtPA = A^T P A
```

迭代最多执行 `160` 次，当矩阵元素最大变化量满足：

```math
\max|P_{\mathrm{next}}-P|<10^{-10}
```

时提前结束。

最终反馈增益为：

```math
K
=
\left(R+B^{T}PB\right)^{-1}B^{T}PA
```

```math
K=
\begin{bmatrix}
K_0 & K_1 & K_2 & K_3
\end{bmatrix}
```

### 5.8 反馈控制律与分段增益

基础反馈量为：

```math
\Delta\phi_{\mathrm{LQR}}
=
-k_s
\left(
K_0e_y
+K_1\dot e_y
+K_2e_\psi
+K_3\dot e_\psi
\right)
```

其中 $k_s$ 由以下参数共同决定：

```text
lqr_feedback_scale
straight_feedback_scale
curve_feedback_scale
```

当前路径满足：

```math
\max
\left(
|\kappa_{\mathrm{now}}|,
|\kappa_{\mathrm{ff}}|
\right)
\leq0.025\ \mathrm{m}^{-1}
```

时被视为直线参考。

默认情况下：

```text
直线反馈缩放：0.55
曲线反馈缩放：1.10
```

直线且不处于路径捕获状态时，LQR 反馈中央转角进一步限制在：

```math
|\Delta\phi_{\mathrm{LQR}}|
\leq0.085\ \mathrm{rad}
```

路径捕获状态下不使用直线或曲线附加缩放，而保留基础 `lqr_feedback_scale`，以获得足够的回归能力。

## 6. 路径曲率与短预瞄转向

### 6.1 当前路径曲率

控制器以路径进度 $s$ 为中心，在默认 $0.18\ \mathrm{m}$ 曲率窗口内获取三个弧长采样点，通过三点外接圆计算有符号曲率：

```math
\kappa
=
\frac{2\left[(p_1-p_0)\times(p_2-p_0)\right]}
{|p_1-p_0|\,|p_2-p_1|\,|p_2-p_0|}
```

曲率最终限制在：

```math
\kappa\in[-\kappa_{\max},\kappa_{\max}]
```

### 6.2 短预瞄距离

短预瞄距离随当前允许速度变化：

```math
D_{\mathrm{short}}
=
\mathrm{clip}
\left(
0.08+0.25|v|,
0.08,
0.38
\right)
```

单位为米。

### 6.3 未来曲率加权平均

控制器在区间：

```math
[s,s+D_{\mathrm{short}}]
```

内使用 `7` 个未来采样点。第 $i$ 个采样比例为：

```math
r_i=\frac{i}{N},\qquad i=1,\ldots,N
```

采样权重为：

```math
w_i=1-0.45r_i
```

越靠近车辆的未来曲率权重越大。未来平均曲率为：

```math
\bar\kappa_{\mathrm{future}}
=
\frac{\sum_{i=1}^{N}w_i\kappa(s+r_iD_{\mathrm{short}})}
{\sum_{i=1}^{N}w_i}
```

### 6.4 平滑激活与融合

定义激活强度：

```math
\kappa_{\mathrm{act}}
=
\max
\left(
|\kappa_{\mathrm{now}}|,
|\bar\kappa_{\mathrm{future}}|
\right)
```

当：

```math
\kappa_{\mathrm{act}}
\leq0.030\ \mathrm{m}^{-1}
```

时，短预瞄不参与融合，直接使用当前曲率。

超过阈值后，激活比例为：

```math
\gamma
=
\mathrm{clip}
\left(
\frac{
\kappa_{\mathrm{act}}-\kappa_{\mathrm{th}}
}{
\max(2\kappa_{\mathrm{th}},0.05)
},
0,
1
\right)
```

有效融合比例为：

```math
\beta_{\mathrm{eff}}
=
\beta\gamma
```

默认 $\beta=0.50$。最终短预瞄曲率为：

```math
\kappa_{\mathrm{preview}}
=
(1-\beta_{\mathrm{eff}})\kappa_{\mathrm{now}}
+
\beta_{\mathrm{eff}}\bar\kappa_{\mathrm{future}}
```

该渐进激活机制可避免曲率刚超过阈值时发生前馈突变。

### 6.5 曲率前馈

前馈曲率为：

```math
\kappa_{\mathrm{ff}}
=
k_{\mathrm{ff}}\kappa_{\mathrm{preview}}
```

默认：

```math
k_{\mathrm{ff}}=1.0
```

对应中央转角：

```math
\phi_{\mathrm{ff}}
=
\arctan
\left(
\frac{L\kappa_{\mathrm{ff}}}{2}
\right)
```

短预瞄对控制器具有两层作用：

1. 直接改变 $\phi_{\mathrm{ff}}$，实现入弯提前转向和出弯提前回正；
2. 通过 $\cos\phi_{\mathrm{ff}}$ 改变输入矩阵 $B$，从而改变本周期在线求解得到的 $K$。

### 6.6 普通路径跟踪前瞻

普通前瞻点与短预瞄曲率不是同一个量。普通路径跟踪前瞻距离为：

```math
D_{\mathrm{track}}
=
\mathrm{clip}
\left(
0.30+0.42|v|,
0.30,
0.85
\right)
```

该前瞻点主要用于参考路径准备和可视化，最近线段误差仍由车辆原点到路径的最近投影计算。

## 7. 长预瞄与速度规划

### 7.1 长预瞄距离

长预瞄只用于速度规划：

```math
D_{\mathrm{long}}
=
\mathrm{clip}
\left(
0.90+1.50|v|,
0.90,
2.50
\right)
```

单位为米。

参数 `curve_anticipation_samples: 20` 表示将预瞄区间划分为 `20` 个间隔。源码从索引 `0` 计算到 `20`，因此实际评估 `21` 个位置。

控制器寻找：

- 第一个满足曲率阈值的位置；
- 该位置到当前投影点的弧长距离 $d_{\mathrm{entry}}$；
- 预瞄范围内的最大绝对曲率 $\kappa_{\mathrm{peak}}$。

默认弯道激活阈值为：

```math
\kappa_{\mathrm{long,th}}=0.050\ \mathrm{m}^{-1}
```

### 7.2 弯道速度上限

弯道目标速度为：

```math
v_{\mathrm{curve}}
=
\mathrm{clip}
\left(
\sqrt{
\frac{a_{y,\mathrm{preview}}}
{|\kappa_{\mathrm{peak}}|}
},
v_{\mathrm{curve,min}},
v_{\max}
\right)
```

默认：

```text
预瞄横向加速度上限：0.12 m/s²
弯道最低速度：       0.24 m/s
```

扣除安全距离后的有效制动距离为：

```math
d_{\mathrm{brake}}
=
\max
\left(
0,
d_{\mathrm{entry}}-d_{\mathrm{safety}}
\right)
```

当前允许速度为：

```math
v_{\mathrm{anticipation}}
=
\sqrt{
 v_{\mathrm{curve}}^2
 +2a_{\mathrm{dec}}d_{\mathrm{brake}}
}
```

并限制在：

```math
v_{\mathrm{anticipation}}
\in
[v_{\mathrm{curve,min}},v_{\max}]
```

未发现弯道时，长预瞄速度上限返回 $v_{\max}$。

### 7.3 用于速度限幅的目标曲率估计

控制器额外构造：

```math
\kappa_{\mathrm{estimate}}
=
\mathrm{clip}
\left(
\kappa_{\mathrm{ff}}
+k_y e_y^{\mathrm{lim}}
+k_\psi e_\psi^{\mathrm{lim}},
-\kappa_{\max},
\kappa_{\max}
\right)
```

默认：

```math
k_y=0.36,\qquad k_\psi=0.66
```

该曲率只用于估计当前转向需求并进行速度限制，实际转向仍由前馈与四状态 DARE-LQR 产生。

### 7.4 当前目标速度

基础目标速度为：

```math
v_0
=
\min
\left(
v_{\mathrm{target}},v_{\max}
\right)
```

曲率限速为：

```math
v_{\mathrm{curvature}}
=
\sqrt{
\frac{a_{y,\max}}
{|\kappa_{\mathrm{estimate}}|}
}
```

跟踪误差速度比例为：

```math
f_e
=
\mathrm{clip}
\left(
\frac{1}
{1+k_{v,y}|e_y|+k_{v,\psi}|e_\psi|},
0.25,
1.0
\right)
```

因此误差限速为：

```math
v_{\mathrm{error}}=f_e v
```

默认：

```math
k_{v,y}=0.55,\qquad k_{v,\psi}=0.38
```

完整速度还会受到以下约束：

- 长预瞄弯道速度上限；
- 路径捕获速度上限；
- 终点纵向减速上限；
- 最大线速度；
- 终端修正最低速度；
- 额外转向需求限速。

源码中还存在一个固定的转向需求限速：

```math
v_{\mathrm{steering}}
=
\sqrt{
\frac{0.35}
{|\kappa_{\mathrm{estimate}}|}
}
```

该常数 `0.35` 当前未暴露为 YAML 参数。

### 7.5 两遍速度与参考计算

由于短预瞄、长预瞄和普通前瞻距离都依赖速度，控制器执行两遍计算：

```text
第一遍：
使用最大目标速度计算参考量
        ↓
根据曲率、误差、终点和路径捕获状态得到第一遍允许速度

第二遍：
使用第一遍允许速度重新计算三个速度相关预瞄尺度
        ↓
重新计算路径曲率、前馈曲率、长预瞄限速和最终速度
```

这样可避免始终使用最大速度预瞄造成低速终点阶段参考尺度过大，也可避免先使用过低速度导致弯道识别距离不足。

## 8. 路径捕获与重新捕获

当车辆初始位置偏离路径较大时，直接以正常速度运行容易出现大幅转向。路径捕获模块使用带滞回的状态机降低速度。

### 8.1 首次进入捕获状态

首次跟踪时，满足以下任一条件进入路径捕获：

```math
|e_y|\geq0.080\ \mathrm{m}
```

或：

```math
|e_\psi|\geq0.220\ \mathrm{rad}
```

### 8.2 跟踪后的重新捕获

车辆已经稳定跟踪过路径后，只有误差增大到以下阈值才重新进入捕获：

```math
|e_y|\geq0.250\ \mathrm{m}
```

或：

```math
|e_\psi|\geq0.550\ \mathrm{rad}
```

这避免正常的小幅误差波动频繁触发低速模式。

### 8.3 退出捕获状态

误差连续 `3` 个控制周期满足：

```math
|e_y|\leq0.025\ \mathrm{m}
```

且：

```math
|e_\psi|\leq0.080\ \mathrm{rad}
```

后退出捕获状态。

捕获速度根据误差严重程度在以下范围内线性插值：

```text
最小捕获速度：0.12 m/s
最大捕获速度：0.55 m/s
```

误差归一化上限默认使用：

```text
横向误差：0.35 m
航向误差：0.80 rad
```

## 9. 终点减速、停车锁存与倒车修正

### 9.1 与 `parking_adjustment_enable` 的关系

LQR 内部终点控制仅在以下条件下启用：

```text
terminal_target_enabled == true
并且
parking_adjustment_enable == false
```

当 `parking_adjustment_enable: true` 时，LQR 不执行自身终点锁存，并清除内部终点状态，由 `nav_core` 的停车细调流程接管。因此，这两种模式的职责为：

```text
parking_adjustment_enable: false
    → 纯路径跟踪控制器自然减速并停车

parking_adjustment_enable: true
    → 路径跟踪阶段由 LQR 完成，最终停车细调由 nav_core 接管
```

### 9.2 终点误差分解

终点在统一运动坐标系中的位置为 $(x_t,y_t)$，终端切线方向为 $\psi_t$。

终点纵向误差为：

```math
e_s
=
\cos\psi_t\,x_t
+
\sin\psi_t\,y_t
```

终点横向误差为：

```math
e_{t,y}
=
-\sin\psi_t\,x_t
+
\cos\psi_t\,y_t
```

终点欧氏距离为：

```math
d_t=\sqrt{x_t^2+y_t^2}
```

### 9.3 终点减速

当：

```math
d_t<2.00\ \mathrm{m}
```

时进入终点减速区域。

纵向速度上限为：

```math
v_{\mathrm{terminal}}
=k_s e_s
```

默认：

```math
k_s=0.50
```

若：

```math
e_s\leq0
```

说明当前 `base_link` 已经到达或越过终端切平面。继续保持当前挡位只会增大纵向误差，因此控制器直接输出零速度，不会为了追逐剩余横向误差而继续越过终点。

### 9.4 低速修正速度下限

终点附近的修正速度下限由横向误差和距离误差共同计算：

```math
v_{\mathrm{floor}}
=
v_{\mathrm{terminal,min}}
+k_{f,y}\max(0,|e_{t,y}|-e_{t,y}^{\mathrm{tol}})
+k_{f,d}\max(0,d_t-d_t^{\mathrm{tol}})
```

默认：

```text
terminal_min_linear_velocity:              0.010 m/s
terminal_lateral_error_speed_floor_gain:   1.80
terminal_distance_error_speed_floor_gain:  0.30
terminal_correction_speed_max:             0.040 m/s
```

修正速度下限不能覆盖纵向制动律，实际使用：

```math
v_{\mathrm{safe,floor}}
=
\min
\left(
v_{\mathrm{floor}},
v_{\mathrm{terminal}}
\right)
```

该限制用于避免车辆在最后几厘米因横向误差仍然存在而被强制保持过高速度，从而越过目标点。

### 9.5 停车位置窗口

当前 YAML 默认停车窗口为：

```math
d_t\leq0.01\ \mathrm{m}
```

```math
|e_s|\leq0.01\ \mathrm{m}
```

```math
|e_{t,y}|\leq0.01\ \mathrm{m}
```

三个条件同时满足时，立即输出零速度并开始停车确认。

### 9.6 停车锁存与滞回

首次进入停车窗口后：

1. 输出零速度；
2. 启动停车保持计数；
3. 在终点距离不超过 `0.03 m` 的滞回范围内继续保持零速度；
4. 连续保持 `3` 个周期后置位 `terminal_stop_latched`。

默认控制周期为 `0.10 s`。锁存需要连续观察到 `3` 个零速保持周期；从首次进入窗口到计数达到 `3` 通常跨越两个后续控制间隔，约为 `0.20 s`，对应三个离散控制采样。

`0.03 m` 的退出阈值用于抑制定位厘米级抖动导致速度在零和非零之间反复切换。

### 9.7 倒车终点横向修正

仅在以下条件同时满足时启用倒车附加修正：

- 当前为倒车；
- 存在真实终点目标；
- 终点距离小于 `1.50 m`；
- 尚未明显越过终点切平面。

倒车修正基于终点横向误差超出死区的部分：

```math
e_{\mathrm{excess}}
=
\max
\left(
0,
|e_{t,y}|-0.004
\right)
```

基础修正为：

```math
\Delta\phi_{\mathrm{reverse}}
=
\mathrm{sgn}(e_{t,y})
\,k_r e_{\mathrm{excess}}
\,b_{\mathrm{entry}}
\,b_{\mathrm{fade}}
```

默认：

```text
倒车终点修正增益：     1.35
最大附加中央转角：     0.035 rad
开始作用距离：         1.50 m
最终衰减距离：         0.08 m
横向误差死区：         0.004 m
```

当横向误差已经进入停车横向窗口时，修正量随终点距离进一步衰减，避免最后阶段仍保持横向转向而破坏终点航向。

正向终点没有对应的额外终点转向项，始终保持：

```math
\phi_{\mathrm{forward}}
=
\phi_{\mathrm{ff}}
+
\Delta\phi_{\mathrm{LQR}}
```

## 10. 曲率与速度指令平滑

### 10.1 曲率平滑

控制器先对最终目标曲率进行变化率限制，再进行一阶平滑。

基础最大曲率变化量为：

```math
\Delta\kappa_{\max}
=
\dot\kappa_{\max}\Delta t
```

默认：

```math
\dot\kappa_{\max}=3.00\ \mathrm{m}^{-1}\mathrm{s}^{-1}
```

处理后的曲率为：

```math
\kappa_{\mathrm{rate}}
=
\kappa_{k-1}
+
\mathrm{clip}
\left(
\kappa_{\mathrm{raw}}-\kappa_{k-1},
-\Delta\kappa_{\max},
\Delta\kappa_{\max}
\right)
```

```math
\kappa_k
=
\alpha_\kappa\kappa_{\mathrm{rate}}
+
(1-\alpha_\kappa)\kappa_{k-1}
```

当前实现中，$\alpha_\kappa$ 越大，当前目标占比越高，响应越快。

根据转向阶段使用不同参数：

```text
普通曲率平滑系数：      0.55
入弯平滑系数：          0.85
出弯平滑系数：          0.72
入弯变化率倍率：        1.40
直线换向平滑系数：      0.30
直线换向变化率倍率：    0.45
```

该分段设计使车辆入弯时能够较快建立转向，出弯时平稳回正，并抑制直线附近曲率正负号频繁切换。

### 10.2 线速度平滑

线速度经过一阶平滑：

```math
v_k
=
\alpha_v v_{\mathrm{target}}
+
(1-\alpha_v)v_{k-1}
```

默认：

```text
速度平滑系数：0.65
减速平滑系数：1.00
```

减速时使用更大的当前目标权重，以保证终点停车和弯前减速不会因平滑产生明显延迟。

加速阶段还受到最大线加速度约束：

```math
|v_k-v_{k-1}|
\leq
 a_{\max}\Delta t
```

默认：

```math
a_{\max}=0.80\ \mathrm{m/s^2}
```

### 10.3 保持转向比值

Ranger 底盘根据：

```math
\frac{\omega}{|v|}
```

解释转向需求。因此线速度平滑后，控制器不独立滤波角速度，而是保持原始角速度与速度大小的比值：

```math
\omega_{\mathrm{smoothed}}
=
|v_{\mathrm{smoothed}}|
\frac{\omega_{\mathrm{raw}}}{|v_{\mathrm{raw}}|}
```

这样可避免只平滑线速度后改变等效转向角。

## 11. 参数配置

参数文件位于：

```text
config/controller.yaml
```

### 11.1 车辆速度与几何参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `target_linear_velocity` | `1.00` | 期望巡航速度，单位：m/s |
| `min_linear_velocity` | `0.06` | 非终点区域最小运行速度，单位：m/s |
| `max_linear_velocity` | `1.00` | 最大线速度，单位：m/s |
| `max_angular_velocity` | `1.20` | 最大角速度指令，单位：rad/s |
| `max_linear_acceleration` | `0.80` | 最大线加速度，单位：m/s² |
| `wheel_base` | `0.494` | 轴距 $L$，单位：m |
| `track_width` | `0.364` | 轮距 $W$，单位：m |
| `max_steering_angle` | `0.698` | 最大内轮转角，单位：rad |
| `min_turning_radius` | `0.59` | 最小转弯半径，单位：m |
| `dual_ackermann_mode_radius_margin` | `0.03` | 双阿克曼半径安全余量，单位：m |

### 11.2 DARE-LQR 参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `lqr_q_lateral` | `12.0` | 横向误差权重 |
| `lqr_q_lateral_rate` | `0.30` | 横向误差变化率权重 |
| `lqr_q_heading` | `10.0` | 航向误差权重 |
| `lqr_q_heading_rate` | `0.25` | 航向误差变化率权重 |
| `lqr_r_steering` | `4.2` | 中央转角修正代价 |
| `lqr_feedback_scale` | `1.00` | LQR 总体反馈缩放 |
| `lqr_error_rate_filter_alpha` | `0.25` | 误差变化率低通系数 |
| `straight_reference_curvature_threshold` | `0.025` | 直线参考曲率阈值，单位：1/m |
| `straight_feedback_scale` | `0.55` | 直线反馈缩放 |
| `curve_feedback_scale` | `1.10` | 曲线反馈缩放 |
| `straight_feedback_max_central_steering` | `0.085` | 直线最大反馈中央转角，单位：rad |
| `lateral_error_limit` | `0.25` | LQR 横向误差限幅，单位：m |
| `heading_error_limit` | `0.70` | LQR 航向误差限幅，单位：rad |
| `lateral_error_deadband` | `0.0015` | 横向误差死区，单位：m |
| `heading_error_deadband` | `0.0020` | 航向误差死区，单位：rad |

### 11.3 曲率前馈、短预瞄和普通前瞻

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `path_curvature_feedforward_gain` | `1.00` | 路径曲率前馈增益 |
| `path_curvature_window_distance` | `0.18` | 三点曲率计算窗口，单位：m |
| `curve_preview_enable` | `true` | 是否启用短预瞄曲率 |
| `curve_preview_min_distance` | `0.08` | 最小短预瞄距离，单位：m |
| `curve_preview_speed_gain` | `0.25` | 短预瞄速度增益 |
| `curve_preview_max_distance` | `0.38` | 最大短预瞄距离，单位：m |
| `curve_preview_samples` | `7` | 未来曲率采样点数 |
| `curve_preview_blend` | `0.50` | 最大短预瞄融合比例 |
| `curve_preview_activation_curvature` | `0.030` | 短预瞄激活曲率，单位：1/m |
| `lookahead_min_distance` | `0.30` | 最小普通前瞻距离，单位：m |
| `lookahead_max_distance` | `0.85` | 最大普通前瞻距离，单位：m |
| `lookahead_speed_gain` | `0.42` | 普通前瞻速度增益 |
| `direction_filter_margin` | `0.08` | 允许保留的车辆后方路径长度，单位：m |
| `minimum_reference_length` | `0.40` | 最小参考路径长度，单位：m |
| `terminal_heading_lookback_distance` | `0.65` | 终端切线回看距离，单位：m |

### 11.4 长预瞄参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `curve_anticipation_enable` | `true` | 是否启用长预瞄减速 |
| `curve_anticipation_min_distance` | `0.90` | 最小长预瞄距离，单位：m |
| `curve_anticipation_speed_gain` | `1.50` | 长预瞄速度增益 |
| `curve_anticipation_max_distance` | `2.50` | 最大长预瞄距离，单位：m |
| `curve_anticipation_samples` | `20` | 长预瞄区间数量，实际评估位置数为 21 |
| `curve_anticipation_curvature_threshold` | `0.050` | 弯道识别阈值，单位：1/m |
| `curve_anticipation_lateral_acceleration` | `0.12` | 长预瞄弯道横向加速度上限，单位：m/s² |
| `curve_anticipation_min_speed` | `0.24` | 长预瞄弯道最低速度，单位：m/s |
| `curve_anticipation_deceleration` | `0.65` | 弯前允许减速度，单位：m/s² |
| `curve_anticipation_safety_distance` | `0.25` | 弯道入口安全距离，单位：m |

### 11.5 路径捕获和速度限制参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `path_acquisition_speed_limit_enable` | `true` | 是否启用路径捕获限速 |
| `path_acquisition_enter_lateral_error` | `0.080` | 首次捕获横向进入阈值，单位：m |
| `path_acquisition_exit_lateral_error` | `0.025` | 捕获横向退出阈值，单位：m |
| `path_acquisition_full_lateral_error` | `0.350` | 横向严重程度归一化上限，单位：m |
| `path_acquisition_enter_heading_error` | `0.220` | 首次捕获航向进入阈值，单位：rad |
| `path_acquisition_exit_heading_error` | `0.080` | 捕获航向退出阈值，单位：rad |
| `path_acquisition_full_heading_error` | `0.800` | 航向严重程度归一化上限，单位：rad |
| `path_reacquisition_enter_lateral_error` | `0.250` | 重新捕获横向进入阈值，单位：m |
| `path_reacquisition_enter_heading_error` | `0.550` | 重新捕获航向进入阈值，单位：rad |
| `path_acquisition_min_speed` | `0.120` | 捕获最小速度，单位：m/s |
| `path_acquisition_max_speed` | `0.550` | 捕获最大速度，单位：m/s |
| `path_acquisition_stable_cycles` | `3` | 退出捕获所需稳定周期数 |
| `curvature_speed_limit_enable` | `true` | 是否启用曲率限速 |
| `max_lateral_acceleration` | `0.32` | 常规曲率限速横向加速度，单位：m/s² |
| `error_speed_limit_enable` | `true` | 是否启用误差限速 |
| `lateral_error_speed_gain` | `0.55` | 横向误差限速增益 |
| `heading_error_speed_gain` | `0.38` | 航向误差限速增益 |
| `speed_curvature_lateral_error_gain` | `0.36` | 速度曲率估计的横向误差增益 |
| `speed_curvature_heading_error_gain` | `0.66` | 速度曲率估计的航向误差增益 |

### 11.6 终点控制参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `terminal_stop_on_path_end_enable` | `true` | 无外部终点时是否使用路径末点停车 |
| `terminal_stop_latch_enable` | `true` | 是否启用停车锁存 |
| `terminal_slowdown_enable` | `true` | 是否启用终点自然减速 |
| `terminal_slowdown_distance` | `2.00` | 终点减速起始距离，单位：m |
| `terminal_stop_distance` | `0.01` | 终点欧氏距离阈值，单位：m |
| `terminal_stop_exit_distance` | `0.030` | 零速保持退出阈值，单位：m |
| `terminal_stop_hold_cycles` | `3` | 锁存确认周期数 |
| `terminal_min_linear_velocity` | `0.010` | 终点最低模型与运动速度基准，单位：m/s |
| `terminal_longitudinal_speed_gain` | `0.50` | 终点纵向减速增益 |
| `terminal_stop_s_tolerance` | `0.01` | 终点纵向误差阈值，单位：m |
| `terminal_stop_lateral_tolerance` | `0.01` | 终点横向误差阈值，单位：m |
| `terminal_correction_min_velocity` | `0.010` | 未满足停车窗口时的最低修正速度，单位：m/s |
| `terminal_correction_speed_max` | `0.040` | 终点修正速度上限，单位：m/s |
| `terminal_lateral_error_speed_floor_gain` | `1.80` | 横向误差对应的修正速度下限增益 |
| `terminal_distance_error_speed_floor_gain` | `0.30` | 距离误差对应的修正速度下限增益 |
| `reverse_terminal_lateral_convergence_enable` | `true` | 是否启用倒车终点横向修正 |
| `reverse_terminal_lateral_convergence_start_distance` | `1.50` | 倒车修正开始距离，单位：m |
| `reverse_terminal_lateral_convergence_fade_distance` | `0.08` | 倒车修正最终衰减距离，单位：m |
| `reverse_terminal_lateral_convergence_deadband` | `0.004` | 倒车横向修正死区，单位：m |
| `reverse_terminal_lateral_convergence_gain` | `1.35` | 倒车横向修正增益 |
| `reverse_terminal_lateral_convergence_max_steering` | `0.035` | 倒车最大附加中央转角，单位：rad |

### 11.7 平滑与周期参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `curvature_smoothing_enable` | `true` | 是否启用曲率平滑 |
| `curvature_smoothing_alpha` | `0.55` | 普通曲率平滑系数 |
| `curve_entry_smoothing_alpha` | `0.85` | 入弯曲率平滑系数 |
| `curve_exit_smoothing_alpha` | `0.72` | 出弯曲率平滑系数 |
| `curve_entry_rate_multiplier` | `1.40` | 入弯曲率变化率倍率 |
| `max_curvature_rate` | `3.00` | 最大曲率变化率，单位：1/(m·s) |
| `straight_sign_change_smoothing_alpha` | `0.30` | 直线曲率换向平滑系数 |
| `straight_sign_change_rate_multiplier` | `0.45` | 直线曲率换向变化率倍率 |
| `command_smoothing_enable` | `true` | 是否启用速度指令平滑 |
| `command_smoothing_alpha` | `0.65` | 速度平滑系数 |
| `command_deceleration_alpha` | `1.00` | 减速阶段平滑系数 |
| `control_period` | `0.10` | DARE 离散周期和控制周期，单位：s |

## 12. 诊断话题

源码使用 ROS 2 私有话题名：

```text
~/lookahead_point
~/tracking_error
```

实际解析后的完整话题名称取决于承载插件的节点名和命名空间，不应固定假设为 `/lqr/...`。可通过以下命令确认：

```bash
ros2 topic list | grep -E 'lookahead_point|tracking_error'
```

### 12.1 前瞻点

```text
~/lookahead_point
```

消息类型：

```text
geometry_msgs/msg/PoseStamped
```

发布普通路径跟踪前瞻点。发布前会从统一运动坐标系转换回实际 `base_link` 方向，因此倒车时显示位置与真实局部路径一致。

### 12.2 跟踪诊断数组

```text
~/tracking_error
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

当前数组固定包含 `24` 个元素：

| 索引 | 字段 | 单位或含义 |
|---:|---|---|
| 0 | `lateral_error` | 横向误差 $e_y$，单位：m |
| 1 | `heading_error` | 航向误差 $e_\psi$，单位：rad |
| 2 | `terminal_distance` | 终点欧氏距离，单位：m |
| 3 | `commanded_linear_velocity` | 发布线速度，单位：m/s |
| 4 | `commanded_yaw_rate` | 发布角速度，单位：rad/s |
| 5 | `K0` | LQR 横向误差增益 |
| 6 | `K1` | LQR 横向误差变化率增益 |
| 7 | `K2` | LQR 航向误差增益 |
| 8 | `K3` | LQR 航向误差变化率增益 |
| 9 | `lookahead_distance` | 当前普通前瞻距离，单位：m |
| 10 | `direction_sign` | 正向为 `+1`，倒向为 `-1` |
| 11 | `terminal_x` | 终点在统一运动坐标系中的 $x$，单位：m |
| 12 | `terminal_y` | 终点在统一运动坐标系中的 $y$，单位：m |
| 13 | `terminal_heading` | 终端切线航向，单位：rad |
| 14 | `reverse_terminal_trim` | 倒车终点附加中央转角，单位：rad |
| 15 | `terminal_is_real_target` | 是否存在有效终点控制目标，`1/0` |
| 16 | `terminal_along_error` | 终点纵向误差 $e_s$，单位：m |
| 17 | `terminal_cross_track_error` | 终点横向误差 $e_{t,y}$，单位：m |
| 18 | `terminal_stop_latched` | 是否完成停车锁存，`1/0` |
| 19 | `current_path_curvature` | 当前路径曲率，单位：1/m |
| 20 | `preview_path_curvature` | 短预瞄融合曲率，单位：1/m |
| 21 | `feedforward_curvature` | 前馈曲率，单位：1/m |
| 22 | `raw_commanded_curvature` | 平滑前目标曲率，单位：1/m |
| 23 | `smoothed_curvature` | 平滑后曲率，单位：1/m |

> `path_acquisition_active` 当前只出现在节流日志中，没有写入 `Float64MultiArray`。如需长期依赖诊断字段，建议后续定义专用 ROS 2 消息，而不是继续扩展无字段名的数组。

### 12.3 终端查看示例

```bash
ros2 topic echo /实际节点命名空间/tracking_error
```

```bash
ros2 topic hz /实际节点命名空间/tracking_error
```

```bash
ros2 topic echo /实际节点命名空间/lookahead_point
```

## 13. 工程集成与编译

### 13.1 目录结构

将本目录放入工作区：

```text
<workspace>/src/controller/lqr
```

主要文件：

```text
lqr/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml
├── config/
│   └── controller.yaml
├── include/lqr/
│   └── lqr.hpp
├── src/
│   └── lqr.cpp
├── assets/
│   └── ranger_mini_v3.jpg
└── README.md
```

### 13.2 插件注册

插件描述：

```xml
<library path="lqr">
  <class
    name="controller/lqr"
    type="lqr::LqrController"
    base_class_type="nav_core::ControllerPlugin">
    <description>Dual-Ackermann four-state DARE-LQR path tracking controller.</description>
  </class>
</library>
```

参数文件中的插件配置：

```yaml
plugin_name: "lqr"
plugins:
  controller: "controller/lqr"
```

### 13.3 依赖

该包依赖：

```text
ament_cmake
geometry_msgs
nav_core
nav_msgs
pluginlib
rclcpp
std_msgs
tf2_ros
```

同时依赖同一工作区 `nav_core` 提供的：

```text
controller_plugin.hpp
robot_pose_transform.hpp
```

其中 `robot_pose_transform.hpp` 用于将定位传感器位姿转换到车辆 `base_link` 参考点。

### 13.4 编译

```bash
cd ~/neu_nav_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select lqr

source install/setup.bash
```

如果 `nav_core` 尚未编译或接口刚刚发生变化，建议直接编译相关依赖：

```bash
cd ~/neu_nav_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to lqr

source install/setup.bash
```

### 13.5 检查安装结果

```bash
ros2 pkg prefix lqr
```

```bash
ls "$(ros2 pkg prefix lqr)/share/lqr"
```

安装目录中应至少包含：

```text
config/
plugin_description.xml
package.xml
```

## 14. 参数调试建议

参数调试应按因果链顺序进行，不建议一开始直接修改 $Q$ 和 $R$。

### 14.1 推荐顺序

1. **车辆几何**：标定 `wheel_base`、`track_width`、最大内轮转角和最小转弯半径；
2. **参考点与外参**：确认里程计对应的传感器坐标系，并验证定位位姿到 `base_link` 的外参方向；
3. **控制周期**：使 `control_period` 与 `nav_core` 实际定时器周期一致；
4. **路径曲率**：检查局部路径采样、曲率符号和终端切线；
5. **曲率前馈**：调整 `path_curvature_feedforward_gain` 和短预瞄参数；
6. **速度规划**：调整曲率限速、长预瞄减速和路径捕获限速；
7. **LQR 权重**：最后调整 $Q$、$R$ 和直线/曲线反馈缩放；
8. **终点参数**：在路径跟踪稳定后调整减速距离、位置窗口和倒车终点修正；
9. **平滑参数**：最后处理入弯、出弯和直线小幅摆动。

### 14.2 常见现象与优先检查项

| 现象 | 优先检查 |
|---|---|
| 入弯明显偏晚 | 增大短预瞄距离、融合比例或曲率前馈增益；不要先盲目增大 LQR 横向权重 |
| 出弯回正偏晚 | 检查短预瞄是否能看到后续直线；适当提高短预瞄作用或出弯响应系数 |
| 直线左右摆动 | 检查路径曲率噪声、外参和控制周期；减小直线反馈缩放或增大 `R` |
| 曲线跟踪偏外侧 | 检查曲率前馈几何、曲率符号和轴距；再适当提高曲线反馈缩放 |
| 第一周期大幅转向 | 检查局部路径是否包含车辆后方点、路径方向是否正确以及捕获状态是否启用 |
| 倒车终点固定横向偏差 | 检查 `base_link` 外参、终端横向误差符号和倒车修正参数 |
| 正向终点突然勾向一侧 | 不应增加正向终点航向覆盖项；检查路径末端切线、前馈曲率和曲率平滑 |
| 终点越过目标后仍前进 | 检查终点是否使用同一 `base_link` 参考点，以及 $e_s\leq0$ 的越界停车逻辑是否启用 |
| 最后几厘米速度忽大忽小 | 检查终点修正速度下限是否超过纵向制动上限，以及锁存退出阈值是否合理 |
| 修改参数无效果 | 确认运行节点实际加载的是 `config/controller.yaml`，并检查安装空间是否已重新构建和重新 `source` |

### 14.3 $Q$、$R$ 的一般影响

- 增大 `lqr_q_lateral`：提高横向误差修正倾向，但可能增加直线摆动；
- 增大 `lqr_q_heading`：提高航向对齐倾向，但过大可能在曲率变化处产生激进转向；
- 增大变化率权重：更重视误差动态，但变化率估计噪声也会被放大；
- 增大 `lqr_r_steering`：减小反馈转角，控制更平缓，但误差收敛更慢；
- 调整 $Q$、$R$ 后应同时观察在线 $K_0$ 到 $K_3$，而不是只观察最终 `angular.z`。

## 15. 使用边界

1. 当前模型属于低速运动学控制，不显式描述轮胎侧偏、执行器延迟、车体侧倾和高速动力学；
2. 双阿克曼几何、底盘角速度接口和 Ranger 控制适配器之间必须保持一致；
3. 局部路径需要具有正确的点序、合理的采样间距和连续的终端切线；
4. `control_period` 必须接近实际控制回调周期，否则离散模型与误差变化率都会失配；
5. 外部终点应表示期望 `base_link` 位姿，不能与定位传感器参考点混用；
6. 当前停车锁存是位置条件，不是位置与航向联合条件；
7. 参数中的厘米级阈值接近定位噪声量级时，应结合实际定位方差设置锁存周期和退出滞回；
8. 不同车辆、载荷、地面摩擦和底盘固件下，需要重新标定几何和响应参数；
9. `Float64MultiArray` 诊断字段没有编译期语义约束，修改源码字段顺序时必须同步更新数据分析程序；
10. 本项目主要用于双阿克曼 DARE-LQR 控制算法的工程实现与设计参考，不能替代目标车辆上的系统辨识和安全测试。

## License

本项目采用 [MIT License](./LICENSE)。
