# Dual-Ackermann DARE-LQR Controller

> 停车精度：小于 1 cm，角度误差小于 1°

> [Ranger Mini V3 效果视频](https://www.xiaohongshu.com/explore/6a51df110000000021018133?xsec_token=AB68jAZqpw4yHAFrqr9d0r3AACLUZA5kRXH2MK-Ds1yCQ=&xsec_source=pc_user)

![Ranger Mini V3](./assets/ranger_mini_v3.jpg)

## 1. 控制流程

```text
局部 base_link 控制路径
        ↓
转换到正、倒车统一运动坐标系
        ↓
最近线段投影：e_y、e_psi、路径进度 s
        ↓
短预瞄：生成转向前馈曲率 kappa_ff
长预瞄：生成前方弯道速度上限 v_curve
        ↓
根据终点距离、路径曲率、跟踪误差和路径捕获状态
共同计算目标速度 |v|
        ↓
使用实际允许速度重新计算速度相关预瞄距离
        ↓
根据 |v| 和 phi_ref 在线构造 A、B，并求解 DARE
        ↓
Delta_phi_LQR = -K [e_y, e_y_dot, e_psi, e_psi_dot]^T
        ↓
phi_cmd = phi_ff + Delta_phi_LQR + reverse_terminal_trim
        ↓
双阿克曼几何换算、曲率平滑和 cmd_vel 发布
```

## 2. 四状态 DARE-LQR

### 2.1 状态向量与控制输入

状态向量定义为：

$$
X_k=
\begin{bmatrix}
e_y &
\dot{e}_y &
e_\psi &
\dot{e}_\psi
\end{bmatrix}^{T}
$$

其中：

- `e_y`：横向位置误差；
- `dot(e_y)`：横向误差变化率；
- `e_psi`：航向角误差；
- `dot(e_psi)`：航向角误差变化率。

控制输入定义为：

$$
u_k=\Delta\phi_k
$$

`Delta_phi` 是相对于路径曲率前馈中央转角的反馈修正量，而不是完整的中央转角指令。

最终中央转角指令为：

$$
\phi_{\mathrm{cmd}}
=
\phi_{\mathrm{ff}}
+
\Delta\phi_{\mathrm{LQR}}
+
\Delta\phi_{\mathrm{reverse}}
$$

其中：

- `phi_ff`：路径曲率前馈中央转角；
- `Delta_phi_LQR`：DARE-LQR 反馈修正量；
- `Delta_phi_reverse`：倒车终点阶段的附加修正量。

### 2.2 离散状态空间模型

离散状态方程为：

$$
X_{k+1}
=
A(v,\Delta t)X_k
+
B(v,\phi_{\mathrm{ref}})\Delta\phi_k
$$

状态矩阵和输入矩阵为：

$$
A=
\begin{bmatrix}
1 & \Delta t & 0 & 0 \\
0 & 0 & v & 0 \\
0 & 0 & 1 & \Delta t \\
0 & 0 & 0 & 0
\end{bmatrix}
$$

$$
B=
\begin{bmatrix}
0 \\
0 \\
0 \\
-\dfrac{2v\cos\left(\phi_{\mathrm{ref}}\right)}{L}
\end{bmatrix}
$$

其中：

- `v`：当前控制周期用于模型构造的速度大小；
- `Delta_t`：控制周期；
- `phi_ref`：LQR 线性化所使用的参考中央转角；
- `L`：车辆轴距。

### 2.3 代价函数

无限时域离散 LQR 的代价函数为：

$$
J=
\sum_{k=0}^{\infty}
\left(
X_k^{T}QX_k
+
u_k^{T}Ru_k
\right)
$$

默认权重为：

$$
Q=
\operatorname{diag}
\left(
12.0,\,
0.30,\,
10.0,\,
0.25
\right)
$$

$$
R=4.2
$$

需要注意：

- `Delta_t` 和 `v` 不属于 `Q` 矩阵；
- `Q` 只包含四个状态量的代价权重；
- `Delta_t` 和 `v` 位于状态矩阵 `A` 和输入矩阵 `B` 中；
- `Q` 和 `R` 决定控制器对状态误差和转向修正量的权衡。

### 2.4 DARE 代码中的矩阵命名

`solveDiscreteLqrGain()` 直接使用与公式一致的变量名：

```text
A
B
Q
R
P
P_next
AtPA
AtPB
BtPA
K
```

离散代数黎卡提方程的迭代形式为：

$$
P_{\mathrm{next}}
=
Q
+
A^{T}PA
-
A^{T}PB
\left(
R+B^{T}PB
\right)^{-1}
B^{T}PA
$$

对于该单输入系统：

$$
B\in\mathbb{R}^{4\times1}
$$

因此：

$$
R+B^{T}PB
$$

为一个标量。

源码中按照公式显式计算：

$$
A^{T}PB
$$

以及：

$$
B^{T}PA
$$

即分别使用：

```text
AtPB = A^T P B
BtPA = B^T P A
```

源码不再使用 `state_matrix`、`input_matrix`、`riccati_state` 等与公式不直接对应的局部变量名，也不使用：

```text
BtPA[i] * BtPA[j]
```

隐式代替 `AtPB`。

### 2.5 `Delta_t` 的来源

`Delta_t` 直接来自参数：

```yaml
control_period: 0.10
```

该参数必须与 `nav_core` 实际控制定时器周期保持一致。

状态向量中的第一项和第三项使用 `Delta_t` 对对应误差变化率进行一步离散积分：

$$
e_{y,k+1}
=
e_{y,k}
+
\Delta t\,\dot{e}_{y,k}
$$

$$
e_{\psi,k+1}
=
e_{\psi,k}
+
\Delta t\,\dot{e}_{\psi,k}
$$

如果实际控制定时器周期发生变化，应同步修改 `control_period`。否则，DARE 内部使用的状态传播速度会与真实控制循环不一致。

### 2.6 `v` 的来源与在线变化

每个控制周期首先根据多项约束计算允许速度大小：

$$
\lvert v\rvert
=
\min
\left(
v_{\mathrm{target}},
v_{\mathrm{curvature}},
v_{\mathrm{error}},
v_{\mathrm{long\text{-}preview}},
v_{\mathrm{terminal}},
v_{\mathrm{acquisition}}
\right)
$$

随后，`solveDiscreteLqrGain()` 使用以下模型速度：

$$
v_{\mathrm{model}}
=
\max
\left(
\lvert v\rvert,\,
\max
\left(
v_{\mathrm{terminal,min}},
0.05
\right)
\right)
$$

其中，最低模型速度为：

$$
0.05\ \mathrm{m/s}
$$

该下限仅用于防止低速时输入矩阵 `B` 接近零，进而导致离散系统失去足够的可控性或数值稳定性。

实际发布速度仍然可以低于：

$$
0.05\ \mathrm{m/s}
$$

由于 `v` 会随路径曲率、终点距离、跟踪误差和前方曲率在线变化，因此：

$$
A_{2,3}=v
$$

会随控制周期变化，同时：

$$
B_4
=
-\frac{2v\cos\left(\phi_{\mathrm{ref}}\right)}{L}
$$

也会随控制周期变化。

因此，DARE 在线求解得到的反馈增益：

$$
K=
\begin{bmatrix}
K_0 & K_1 & K_2 & K_3
\end{bmatrix}
$$

同样会随当前速度和参考转角变化。

## 3. 短预瞄如何作用于 LQR

### 3.1 短预瞄距离

短预瞄距离按照当前速度动态计算：

$$
D_{\mathrm{short}}
=
\operatorname{clip}
\left(
0.08+0.25\lvert v\rvert,\,
0.08,\,
0.38
\right)
$$

单位为米。

控制器在以下路径区间内采样曲率：

$$
\left[
s,\,
s+D_{\mathrm{short}}
\right]
$$

控制器共获取 7 个曲率采样点，并对距离车辆更近的采样点赋予更大的权重，从而得到未来平均曲率：

$$
\bar{\kappa}_{\mathrm{future}}
$$

只有当当前曲率或未来曲率超过激活阈值时，短预瞄才参与前馈曲率计算。

激活阈值为：

$$
0.030\ \mathrm{m}^{-1}
$$

当前曲率和未来平均曲率经过平滑融合后得到：

$$
\kappa_{\mathrm{preview}}
=
\left(
1-\beta
\right)
\kappa_{\mathrm{now}}
+
\beta
\bar{\kappa}_{\mathrm{future}}
$$

然后计算前馈曲率：

$$
\kappa_{\mathrm{ff}}
=
k_{\mathrm{ff}}
\kappa_{\mathrm{preview}}
$$

根据双阿克曼等效几何关系，将前馈曲率转换为中央转角：

$$
\phi_{\mathrm{ff}}
=
\arctan
\left(
\frac{L\kappa_{\mathrm{ff}}}{2}
\right)
$$

### 3.2 对 LQR 的直接影响

短预瞄对 LQR 有两层直接作用。

#### 第一层：改变曲率前馈转角

短预瞄通过改变 `phi_ff`，使车辆能够：

- 在进入弯道前提前建立转向；
- 在离开弯道前提前开始回正；
- 降低因纯反馈滞后造成的入弯延迟和出弯超调。

#### 第二层：改变 LQR 线性化工作点

前馈中央转角 `phi_ff` 作为线性化参考转角进入输入矩阵 `B`：

$$
B_4
=
-\frac{2v\cos\left(\phi_{\mathrm{ff}}\right)}{L}
$$

因此，短预瞄不仅会叠加前馈转角，还会改变本控制周期内 DARE 所使用的转向响应模型和反馈增益。

## 4. 长预瞄如何作用于 LQR

### 4.1 长预瞄距离

长预瞄距离按照当前速度动态计算：

$$
D_{\mathrm{long}}
=
\operatorname{clip}
\left(
0.90+1.50\lvert v\rvert,\,
0.90,\,
2.50
\right)
$$

单位为米。

控制器在该范围内获取 20 个曲率采样点，并寻找：

- 第一个超过弯道激活阈值的路径位置；
- 对应的弯道入口距离；
- 前方最大绝对曲率 `kappa_peak`。

弯道激活阈值为：

$$
0.050\ \mathrm{m}^{-1}
$$

### 4.2 弯道目标速度

根据允许的最大横向加速度计算弯道目标速度：

$$
v_{\mathrm{curve}}
=
\sqrt{
\frac{
a_{y,\max}
}{
\left|
\kappa_{\mathrm{peak}}
\right|
}
}
$$

其中：

- `a_y,max`：允许的最大横向加速度；
- `kappa_peak`：长预瞄范围内的最大绝对曲率。

### 4.3 当前允许速度

根据弯道目标速度、允许减速度和剩余制动距离，计算当前允许速度：

$$
v_{\mathrm{allow}}
=
\sqrt{
v_{\mathrm{curve}}^{2}
+
2a_{\mathrm{dec}}d_{\mathrm{brake}}
}
$$

其中：

- `a_dec`：允许的纵向减速度；
- `d_brake`：车辆当前位置到弯道入口的有效制动距离。

### 4.4 对 LQR 的间接影响

长预瞄不直接改变中央转角，也不会直接叠加到 LQR 输出中。

长预瞄首先限制目标速度。速度降低后，会进一步影响：

- 状态矩阵 `A` 中的 `v`；
- 输入矩阵 `B` 中的 `v`；
- DARE 在线求解得到的反馈增益 `K`；
- 短预瞄距离；
- 普通跟踪前瞻距离；
- 实车转向指令的变化速度；
- 车辆横向加速度。

短预瞄和长预瞄的作用关系可以概括为：

```text
短预瞄：
直接影响转向前馈和 LQR 线性化工作点

长预瞄：
先限制目标速度，再通过速度 v 间接影响 A、B、K 和预瞄尺度
```

## 5. 参数文件

控制器参数按照功能分组存放在：

```text
config/controller.yaml
```

修改参数时，建议按照以下顺序进行调整：

1. 车辆几何参数和中央转角限制；
2. 曲率前馈参数和预瞄参数；
3. 速度规划参数；
4. 状态权重 `Q`、控制权重 `R` 和反馈缩放系数；
5. 误差变化率滤波和转向指令平滑参数；
6. 终点停车相关参数。

不建议在车辆几何参数尚未校准时直接调整 `Q` 和 `R`，否则可能使用反馈增益补偿错误的运动学模型。

## 6. 诊断话题

### 6.1 前瞻点

```text
/lqr/lookahead_point
```

该话题发布当前普通路径跟踪前瞻点。

### 6.2 跟踪诊断数组

```text
/lqr/tracking_error
```

该话题发布控制诊断数组，各索引含义如下：

| 索引 | 字段 | 单位或含义 |
|---:|---|---|
| 0 | `lateral_error` | 横向误差，单位：m |
| 1 | `heading_error` | 航向角误差，单位：rad |
| 2 | `terminal_distance` | 到终点的欧氏距离，单位：m |
| 3 | `terminal_along_error` | 终点纵向误差，单位：m |
| 4 | `terminal_cross_track_error` | 终点横向误差，单位：m |
| 5 | `commanded_linear_velocity` | 发布的线速度，单位：m/s |
| 6 | `commanded_yaw_rate` | 发布的角速度，单位：rad/s |
| 7 | `commanded_curvature` | 发布指令对应的曲率，单位：1/m |
| 8 | `direction_sign` | 运动方向符号 |
| 9 | `acquisition_active` | 是否处于路径捕获阶段 |
| 10 | `terminal_stop_latched` | 是否已经锁存终点停车状态 |
| 11 | `reverse_terminal_trim` | 倒车终点修正转角，单位：rad |
| 12 | `lookahead_distance` | 当前普通前瞻距离，单位：m |
| 13 | `K0` | LQR 增益第 1 项 |
| 14 | `K1` | LQR 增益第 2 项 |
| 15 | `K2` | LQR 增益第 3 项 |
| 16 | `K3` | LQR 增益第 4 项 |
| 17 | `feedforward_central_steering` | 前馈中央转角，单位：rad |
| 18 | `feedback_central_steering` | LQR 反馈中央转角，单位：rad |
| 19 | `current_path_curvature` | 当前路径曲率，单位：1/m |
| 20 | `preview_path_curvature` | 短预瞄路径曲率，单位：1/m |
| 21 | `feedforward_curvature` | 前馈曲率，单位：1/m |
| 22 | `raw_commanded_curvature` | 未平滑的目标曲率，单位：1/m |
| 23 | `smoothed_curvature` | 平滑后的目标曲率，单位：1/m |

## 7. 编译

将本目录放置到工作区中的以下位置：

```text
src/controller/lqr
```

执行编译：

```bash
cd ~/workspace

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select lqr

source install/setup.bash
```

该插件依赖同一工作区中的 `nav_core`，主要包括：

```text
controller_plugin.hpp
robot_pose_transform.hpp
```

因此，本项目主要用于提供双阿克曼 DARE-LQR 控制算法的设计和实现参考。实际使用时，需要结合目标车辆的以下条件进行调整：

- 车辆轴距和双阿克曼转向几何；
- 最大中央转角和最大曲率；
- 底盘速度与转角响应特性；
- 控制定时器周期；
- 路径坐标系和车辆参考点；
- 定位外参；
- 正向和倒向运动符号；
- 终点位置与航向精度要求。