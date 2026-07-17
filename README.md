# Dual-Ackermann Four-State DARE-LQR Controller

面向 ROS 2 `nav_core` 框架的双阿克曼路径跟踪控制器。控制器以 `base_link` 为统一跟踪与停车参考点，采用四状态离散代数黎卡提方程（Discrete Algebraic Riccati Equation, DARE）在线求解反馈增益，并组合路径曲率前馈、短/长预瞄、底盘执行模型补偿、平滑路径并入、正倒车统一运动坐标系、终点单调减速与厘米级停车锁存。

> 当前默认配置面向 Ranger Mini V3 双阿克曼模式：巡航速度上限 `1.0 m/s`，停车位置窗口 `1 cm`。停车判据只包含位置量 `dist、s、cte`，不包含独立航向阈值；最终航向由末端路径切线、曲率前馈和闭环反馈在运动过程中保证。

> [Ranger Mini V3 效果视频](https://www.xiaohongshu.com/explore/6a51df110000000021018133?xsec_token=AB68jAZqpw4yHAFrqr9d0r3AACLUZA5kRXH2MK-Ds1yCQ=&xsec_source=pc_user)

![Ranger Mini V3](./assets/ranger_mini_v3.jpg)

## 目录

- [1. 功能边界与主要特性](#1-功能边界与主要特性)
- [2. 软件包结构与接口](#2-软件包结构与接口)
- [3. 完整控制流程](#3-完整控制流程)
- [4. 坐标系、参考点与误差定义](#4-坐标系参考点与误差定义)
- [5. 双阿克曼几何与命令接口](#5-双阿克曼几何与命令接口)
- [6. 路径参考、曲率与预瞄](#6-路径参考曲率与预瞄)
- [7. 四状态 DARE-LQR 详细推导](#7-四状态-dare-lqr-详细推导)
- [8. 在线误差变化率与分段反馈](#8-在线误差变化率与分段反馈)
- [9. 底盘数据驱动补偿](#9-底盘数据驱动补偿)
- [10. 速度规划](#10-速度规划)
- [11. 平滑并入、重新捕获与终点交接](#11-平滑并入重新捕获与终点交接)
- [12. 正倒车稳定、终点自然驶入与停车](#12-正倒车稳定终点自然驶入与停车)
- [13. 完整参数表与调参方法](#13-完整参数表与调参方法)
- [14. 诊断话题与日志](#14-诊断话题与日志)
- [15. 推荐调试流程](#15-推荐调试流程)
- [16. 常见问题定位](#16-常见问题定位)
- [17. 编译、运行与参数检查](#17-编译运行与参数检查)
- [18. 使用边界](#18-使用边界)
- [License](#license)

## 1. 功能边界与主要特性

### 1.1 主要特性

- **四状态在线 DARE-LQR**：每个控制周期根据本周期允许速度和补偿后的曲率前馈重新构造离散系统矩阵，并在线迭代求解反馈增益。
- **前馈与反馈解耦**：底盘角速度执行增益只补偿路径曲率前馈，不放大 LQR 反馈和定位噪声。
- **正倒车统一运动坐标系**：正向和倒向共用最近线段投影、路径曲率、DARE-LQR 和停车误差定义。
- **双层预瞄**：短预瞄用于提前建立/释放转向；长预瞄只用于弯前速度规划。
- **平滑路径并入**：大位置/朝向偏差时生成有界渐近航向，并配合反馈降权和速度门控，减少穿越路径后的 S 型反复修正。
- **底盘适配集中**：几何、静态增益、方向延迟、横摆动态、制动模型和倒向差异均集中在 YAML 前半部分，便于更换底盘。
- **出弯恢复和短末段对齐**：转弯后按实际行驶距离增强直线恢复；进入真实终点区域后路径捕获逻辑强制退出，避免破坏停车。
- **自然终点速度**：最后 2 m 由终端切线纵向距离连续限速，最后 2.2 m 的速度上限只允许下降，避免减速后再次加速。
- **统一 `base_link` 终点**：当前定位传感器位姿只应用一次外参转换，目标点直接解释为期望 `base_link` 位姿。
- **正倒向 1 cm 停车窗口**：`dist、|s|、|cte|` 同时不超过 `0.01 m` 时立即输出零速度并锁存。

### 1.2 不应误解的边界

1. 本控制器是低速运动学/工程辨识混合模型，不是完整轮胎动力学控制器。
2. 当前停车条件不单独检查航向误差；路径末段必须提供正确、连续的终端切线。
3. `cmd_vel.angular.z` 按当前双阿克曼适配器约定生成，不能直接套用普通差速或单车模型接口。
4. 底盘经验参数不能替代正确的机械几何、坐标外参和路径质量。
5. 厘米级停车阈值接近定位噪声时，应先评估定位重复性，再决定是否收紧阈值或增加保持周期。

## 2. 软件包结构与接口

### 2.1 目录结构

```text
lqr/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── package.xml
├── plugin_description.xml
├── assets/
│   └── ranger_mini_v3.jpg
├── config/
│   └── controller.yaml
├── include/lqr/
│   └── lqr.hpp
└── src/
    └── lqr.cpp
```

### 2.2 插件接口

插件注册类型：

```text
name: controller/lqr
type: lqr::LqrController
base: nav_core::ControllerPlugin
```

控制器由 `nav_core` 调用，核心输入包括：

- 当前里程计 `nav_msgs/msg/Odometry`；
- 局部 `base_link` 控制路径 `nav_msgs/msg/Path`；
- 地图坐标系下期望 `base_link` 终点 `geometry_msgs/msg/PoseStamped`；
- 正倒车方向符号；
- 是否由外部停车细调模块接管。

输出：

- `geometry_msgs/msg/Twist` 速度指令；
- 私有前瞻点话题；
- 私有控制诊断数组；
- 节流控制日志和一次性最终停车日志。

### 2.3 `nav_core` 与 LQR 的职责

```text
nav_core
  ├─ 提供 robot_base 几何/外参/方向许可
  ├─ 提供局部 base_link 控制路径和最终 base_link 目标
  ├─ 调用 LQR 生成 cmd_vel
  └─ 管理 Cruise / Parking / Idle 等上层状态

LQR
  ├─ 计算路径和终点误差
  ├─ 规划曲率与速度
  ├─ 在线求解 DARE-LQR
  ├─ 处理正倒车和底盘补偿
  └─ 满足停车窗口时输出零速度
```

`robot_base.extrinsics` 不在本包 `controller.yaml` 中，而由 `nav_core` 的 `RobotBaseConfig` 注入。更换定位传感器或 `base_link` 外参时，应修改 `nav_core` 对应底盘配置，不应在 LQR 中重复平移路径。

## 3. 完整控制流程

```text
地图下定位传感器位姿
        ↓ robot_base.extrinsics（只应用一次）
当前 base_link 位姿
        ↓
局部 base_link 控制路径 + 目标 base_link 终点
        ↓
正/倒车统一运动坐标系
        ↓
后方点过滤、短路径延长、必要时构造备用切线路径
        ↓
最近线段投影：e_y、e_psi、路径进度 s_path
        ↓
当前曲率 kappa_now
短预瞄曲率 kappa_preview + 方向延迟附加预瞄
        ↓
名义前馈 kappa_ff_nominal
        ↓ 底盘角速度执行增益逆模型
补偿前馈 kappa_ff
        ↓
长预瞄弯道速度上限
当前曲率速度上限
误差速度上限
路径捕获/出弯恢复/倒向稳定速度上限
终点比例速度 + 制动包络 + 单调速度包络
        ↓
按实际允许速度重新计算第二遍参考
        ↓
在线构造 A(v,dt)、B(v,phi_ff)
        ↓
固定点迭代求解 DARE，得到 K=[K0,K1,K2,K3]
        ↓
中央转角 = 曲率前馈 + LQR反馈 + 直线偏置 + 终点小幅修正
        ↓
曲率死区、反号抑制、曲率变化率限制和一阶平滑
        ↓
中央转角 → 内轮转角 → cmd_vel.angular.z
        ↓
线速度加减速限制与平滑，并保持 angular.z / |linear.x| 比值
        ↓
严格 base_link 停车判定
        ↓
发布 cmd_vel 和诊断信息
```

## 4. 坐标系、参考点与误差定义

### 4.1 当前 `base_link` 位姿

里程计可以表达雷达、IMU 或其他定位传感器的位姿。控制器调用 `robotBasePoseFromOdometry()`：

```text
map_T_sensor × sensor_T_base = map_T_base
```

该外参只应用一次。最终误差始终是：

```text
目标 base_link - 当前 base_link
```

### 4.2 正倒车统一运动坐标系

定义方向符号：

- 正向行驶：$\sigma=+1$；
- 倒向行驶：$\sigma=-1$。

局部路径点转换为：

$$
x_m=\sigma x_b,\qquad y_m=\sigma y_b
$$

因此两个方向在运动坐标系中都等价为沿正 $x_m$ 方向前进。倒车终点航向额外补偿 $\pi$，使运动切线方向保持统一。

### 4.3 最近线段投影

对每条有效线段 $p_0\rightarrow p_1$，将运动坐标系原点投影到线段上。设单位切向量为 $\mathbf t$，单位左法向为 $\mathbf n$，投影点为 $p^*$，则：

$$
e_y=\mathbf n^T p^*
$$

$$
e_\psi=\mathrm{wrap}(\psi_{path})
$$

当前符号约定：

- $e_y>0$：路径在车辆运动方向左侧；
- $e_y<0$：路径在车辆运动方向右侧；
- $e_\psi>0$：路径切线相对车辆运动方向逆时针偏转。

### 4.4 终点误差

终点在运动坐标系中的位置为 $(x_t,y_t)$，终端路径切线为 $\psi_t$：

$$
d_t=\sqrt{x_t^2+y_t^2}
$$

$$
e_s=\cos\psi_t\,x_t+\sin\psi_t\,y_t
$$

$$
e_{t,y}=-\sin\psi_t\,x_t+\cos\psi_t\,y_t
$$

其中：

- $d_t$：目标点欧氏距离；
- $e_s$：沿终端切线的剩余纵向距离；
- $e_{t,y}$：相对终端切线的横向误差。

当前停车条件：

$$
d_t\le 0.01\ \mathrm m,\qquad |e_s|\le0.01\ \mathrm m,\qquad |e_{t,y}|\le0.01\ \mathrm m
$$

### 4.5 路径预处理

1. 运动坐标系中仅保留满足 `x_m >= -direction_filter_margin` 的点；
2. 过滤后少于 2 点则恢复原路径；
3. 有效参考长度不足时沿终端切线延长；
4. 输入路径不足时，以外部目标为中心构造前后切线备用路径；
5. 终端切线默认从末端向前回看 `terminal_heading_lookback_distance` 计算。

## 5. 双阿克曼几何与命令接口

### 5.1 几何变量

- $L$：轴距；
- $W$：轮距；
- $\delta_{in}$：内侧车轮转角；
- $\phi$：等效中央转角；
- $\kappa$：车辆轨迹曲率。

### 5.2 内轮转角与中央转角

源码使用：

$$
\phi=\mathrm{sgn}(\delta_{in})\mathrm{atan2}
\left(
L\sin|\delta_{in}|,
L\cos|\delta_{in}|+W\sin|\delta_{in}|
\right)
$$

反向换算：

$$
\delta_{in}=\mathrm{sgn}(\phi)\mathrm{atan2}
\left(
L\sin|\phi|,
L\cos|\phi|-W\sin|\phi|
\right)
$$

### 5.3 中央转角与曲率

$$
\kappa=\frac{2\tan\phi}{L}
$$

$$
\phi=\arctan\left(\frac{L\kappa}{2}\right)
$$

### 5.4 最大可用曲率

根据最小转弯半径和安全余量先计算允许内轮转角：

$$
\delta_{radius}=\arctan\left(
\frac{L/2}{R_{min}+\Delta R}
\right)
$$

$$
\delta_{usable}=\min(\delta_{max},\delta_{radius})
$$

再换算最大中央转角和最大曲率。源码会对所有目标曲率执行该上限约束。

### 5.5 `cmd_vel.angular.z`

源码先将曲率换算为中央转角和内轮转角，再生成：

$$
\omega_{cmd}=\mathrm{clip}
\left(
\frac{2|v|\tan\delta_{in}}{L},
-\omega_{max},\omega_{max}
\right)
$$

这里使用 $|v|$，正倒车方向由 `linear.x` 符号表达。当前底盘适配器依据 `angular.z/|linear.x|` 还原转向需求，所以速度平滑后必须保持该比值。更换底盘时首先确认新接口语义；若新底盘将 `angular.z` 直接解释为刚体横摆角速度，必须重新检查这一转换层。

## 6. 路径参考、曲率与预瞄

### 6.1 当前曲率

控制器按弧长取得三点，利用外接圆计算有符号曲率：

$$
\kappa=\frac{2[(p_1-p_0)\times(p_2-p_0)]}
{|p_1-p_0|\,|p_2-p_1|\,|p_2-p_0|}
$$

曲率窗口由 `path_curvature_window_distance` 控制，源码实际跨度不小于 `0.12 m`。相邻弧长间隔小于 `0.025 m` 时返回零，避免退化三点造成数值爆炸。

### 6.2 短预瞄

$$
D_{short}=\mathrm{clip}
(D_{min}+k_v|v|,D_{min},D_{max})
$$

未来曲率采样权重：

$$
w_i=1-0.45\frac{i}{N}
$$

未来均值：

$$
\bar\kappa_{future}=\frac{\sum_iw_i\kappa_i}{\sum_iw_i}
$$

只有当前或未来曲率超过 `curve_preview_activation_curvature` 时才逐渐激活融合：

$$
\kappa_{preview}=(1-\beta_{eff})\kappa_{now}+\beta_{eff}\bar\kappa_{future}
$$

### 6.3 方向延迟附加预瞄

物理转向方向符号：

$$
s_{steer}=\sigma\kappa
$$

选择对应延迟 $\tau_+$ 或 $\tau_-$，附加预瞄距离：

$$
D_{delay}=\mathrm{clip}
(|v|\tau\lambda_\tau,0,D_{delay,max})
$$

该方法通过更早读取路径曲率补偿执行延迟，而不是在偏离后放大反馈。

### 6.4 前馈执行增益补偿

名义前馈：

$$
\kappa_{ff,nom}=k_{ff}\kappa_{preview}
$$

由名义前馈计算名义角速度，查表得到执行增益 $g_\omega$。补偿倍率：

$$
s_{ff}=\mathrm{clip}
\left[
1+\lambda_{ff}\left(\frac{1}{g_\omega}-1\right),
1,s_{ff,max}
\right]
$$

$$
\kappa_{ff}=s_{ff}\kappa_{ff,nom}
$$

仅 $\kappa_{ff}$ 被补偿，LQR 反馈量不乘该倍率。

## 7. 四状态 DARE-LQR 详细推导

### 7.1 状态和输入

$$
X_k=\begin{bmatrix}
e_y & \dot e_y & e_\psi & \dot e_\psi
\end{bmatrix}^T
$$

控制输入不是完整转角，而是相对前馈工作点的中央转角修正：

$$
u_k=\Delta\phi_k
$$

最终中央转角：

$$
\phi_{cmd}=\phi_{ff}+\Delta\phi_{LQR}+\phi_{bias}+\phi_{terminal}
$$

### 7.2 当前源码采用的离散误差模型

每周期根据当前模型速度和前馈中央转角重新构造：

$$
X_{k+1}=AX_k+Bu_k
$$

$$
A=\begin{bmatrix}
1&\Delta t&0&0\\
0&0&v_m&0\\
0&0&1&\Delta t\\
0&0&0&0
\end{bmatrix}
$$

$$
B=\begin{bmatrix}
0 \\
0 \\
0 \\
-g_\phi
\end{bmatrix},\qquad
g_\phi=\frac{2v_m\cos\phi_{ff}}{L}
$$

其中：

$$
v_m=\max(|v_{cmd}|,\max(v_{terminal,min},0.05))
$$

> 重要：精确双阿克曼曲率关系对中央转角的数学导数包含 $\sec^2\phi$。当前源码在 DARE 输入矩阵中采用的是工程等效灵敏度 `2 v cos(phi_ff) / L`，README 按实际代码说明。若要更换该线性化模型，必须修改源码并重新验证闭环稳定性，不能仅通过 YAML 调参等价替代。

负号来自当前路径误差和转向方向约定。

### 7.3 无限时域离散二次型代价

$$
J=\sum_{k=0}^{\infty}
\left(X_k^TQX_k+u_k^TRu_k\right)
$$

$$
Q=\mathrm{diag}(q_y,q_{\dot y},q_\psi,q_{\dot\psi})
$$

$$
R=r_\phi>0
$$

当前默认：

$$
Q=\mathrm{diag}(12.0,0.30,10.0,0.25),\qquad R=4.2
$$

### 7.4 从 Bellman 方程推导最优控制律

假设无限时域最优价值函数为：

$$
V(X)=X^TPX
$$

Bellman 方程：

$$
V(X_k)=\min_{u_k}
\left[
X_k^TQX_k+u_k^TRu_k+V(AX_k+Bu_k)
\right]
$$

展开未来价值：

$$
(AX+Bu)^TP(AX+Bu)
=X^TA^TPAX+2u^TB^TPAX+u^TB^TPBu
$$

因此与 $u$ 有关的项为：

$$
u^T(R+B^TPB)u+2u^TB^TPAX
$$

对 $u$ 求导并令其为零：

$$
2(R+B^TPB)u+2B^TPAX=0
$$

得到：

$$
u^*=-KX
$$

$$
K=(R+B^TPB)^{-1}B^TPA
$$

### 7.5 DARE

将 $u^*=-KX$ 代回 Bellman 方程：

$$
P=Q+A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA
$$

这就是离散代数黎卡提方程。

### 7.6 源码固定点迭代

初值：

$$
P_0=Q
$$

迭代：

$$
P_{i+1}=Q+A^TP_iA-A^TP_iB(R+B^TP_iB)^{-1}B^TP_iA
$$

当前实现：

- 最多迭代 `160` 次；
- 元素最大变化量小于 $10^{-10}$ 时提前停止；
- 单输入系统中 $S=R+B^TPB$ 为标量；
- 每个控制周期重新求解，不缓存固定 K。

最终：

$$
K=\begin{bmatrix}K_0&K_1&K_2&K_3\end{bmatrix}
$$

$$
\Delta\phi_{LQR}=-k_{scale}
(K_0e_y+K_1\dot e_y+K_2e_\psi+K_3\dot e_\psi)
$$

## 8. 在线误差变化率与分段反馈

### 8.1 模型变化率

$$
\dot e_{y,model}=|v|\sin e_\psi
$$

$$
\dot e_{\psi,model}=|v|(\kappa_{ff}-\kappa_{prev})
$$

### 8.2 测量差分

$$
\dot e_{y,meas}=\frac{e_{y,k}-e_{y,k-1}}{\Delta t}
$$

$$
\dot e_{\psi,meas}=\frac{\mathrm{wrap}(e_{\psi,k}^{ctrl}-e_{\psi,k-1}^{ctrl})}{\Delta t}
$$

直线测量融合比例为 `0.12`，曲线为 `0.30`。融合后再使用：

$$
\dot e_k^f=\alpha_r\dot e_k^{sample}+(1-\alpha_r)\dot e_{k-1}^f
$$

变化率限幅：

$$
\dot e_y\in[-1,1]\ \mathrm{m/s},\qquad
\dot e_\psi\in[-2,2]\ \mathrm{rad/s}
$$

### 8.3 分段反馈缩放

基础缩放为 `lqr_feedback_scale`。随后按状态选择：

```text
路径捕获：× path_acquisition_feedback_scale
曲线：    × curve_feedback_scale
终点对齐：× forward/reverse terminal_alignment_feedback_scale
出弯恢复：× post_curve_feedback_scale
倒向稳定：× reverse_straight_feedback_scale
普通直线：× straight_feedback_scale
```

路径捕获还会缩放横向状态本身，并限制最大反馈中央转角。普通直线、倒向稳定和终点对齐具有不同的反馈转角上限。

## 9. 底盘数据驱动补偿

### 9.1 静态前馈逆模型

见第 6.4 节。其目的为消除可重复的角速度幅值压缩，而不是提高反馈增益。

### 9.2 方向相关延迟

见第 6.3 节。倒车时物理转向方向与运动坐标系曲率符号的映射会翻转，源码使用 `direction × curvature` 选择延迟表。

### 9.3 横摆动态约束

根据当前状态选择横摆角加速度上限 $\dot\omega_{max}$，换算为曲率变化率：

$$
\dot\kappa_{emp}=\frac{\dot\omega_{max}}{\max(|v|,v_{floor})}
$$

最终允许曲率变化率为基础曲率变化率和实测动态限制的较小值。

### 9.4 制动模型

实测模型形式：

$$
d_{stop}=a_bv^2+b_bv
$$

给定可用距离 $d$，源码反解正根：

$$
v_{brake}=\frac{-b_b+\sqrt{b_b^2+4a_bd}}{2a_b}
$$

该限制只在 `signed_s >= empirical_braking_activation_distance` 时启用，最后精细驶入区只使用连续比例终点速度，避免在横向误差尚未达标时提前压成零速。

## 10. 速度规划

### 10.1 基础速度

$$
v_0=\min(v_{target},v_{max})
$$

### 10.2 当前曲率限速

$$
v_{curv}=\sqrt{\frac{a_{y,max}}{\max(|\kappa_{estimate}|,\epsilon)}}
$$

速度限幅用曲率估计：

$$
\kappa_{estimate}=\mathrm{clip}
(\kappa_{ff}+k_y e_y+k_\psi e_\psi)
$$

该估计只用于速度，不直接替代 DARE-LQR 转向。

### 10.3 误差限速

$$
f_e=\mathrm{clip}
\left(
\frac{1}{1+k_{v,y}|e_y|+k_{v,\psi}|e_\psi|},
0.25,1
\right)
$$

$$
v\leftarrow f_ev
$$

### 10.4 长预瞄弯前限速

长预瞄距离：

$$
D_{long}=\mathrm{clip}(D_{min}+k_v|v|,D_{min},D_{max})
$$

找到前方首次弯道入口距离 $d_{entry}$ 和峰值曲率 $\kappa_{peak}$：

$$
v_{curve}=\mathrm{clip}
\left(
\sqrt{\frac{a_{y,preview}}{|\kappa_{peak}|}},
v_{curve,min},v_{max}
\right)
$$

$$
v_{anticipation}=\sqrt{v_{curve}^2+2a_{dec}\max(0,d_{entry}-d_{safe})}
$$

### 10.5 两遍参考计算

第一遍以最大目标速度建立预瞄尺度并计算初步允许速度；第二遍使用初步允许速度重新计算短/长预瞄和最终控制量，避免低速终点仍使用高速预瞄，也避免始终用低速预瞄漏检远处弯道。

### 10.6 其他速度上限

完整速度取以下上限的最小值：

- 巡航最大速度；
- 当前曲率限速；
- 误差限速；
- 长预瞄弯道限速；
- 路径捕获限速；
- 出弯恢复限速；
- 倒向稳定限速；
- 终点对齐限速（当前默认关闭）；
- 终点纵向比例速度；
- 实测制动速度；
- 源码固定转向需求限速 $\sqrt{0.35/|\kappa_{estimate}|}$。

## 11. 平滑并入、重新捕获与终点交接

### 11.1 捕获状态机

首次任务依据 `path_acquisition_enter_*` 进入捕获；已经跟踪稳定后，依据更大的 `path_reacquisition_enter_*` 重新捕获。退出需要连续 `path_acquisition_stable_cycles` 满足退出阈值。

### 11.2 渐近并入航向

仅参考曲率较小时：

$$
e_{\psi,target}^{acq}=k_{acq}
\mathrm{atan2}\left(-e_{y,excess},L_{merge}\right)
$$

$$
e_{\psi}^{ctrl}=e_\psi-e_{\psi,target}^{acq}
$$

同时：

- 横向状态乘 `path_acquisition_lateral_feedback_scale`；
- 总反馈乘 `path_acquisition_feedback_scale`；
- 反馈中央转角限制为 `path_acquisition_max_central_steering`；
- 速度按误差严重程度在捕获最小/最大速度之间插值。

### 11.3 终点交接

当真实终点满足：

```text
terminal_distance <= terminal_monotonic_release_distance
terminal_along_error >= -terminal_stop_s_tolerance
```

源码立即：

```text
path_acquisition_active = false
path_acquired_once = true
stable_count = 0
```

交接发生在倒向稳定状态更新之前，因此同一控制周期即可由终点对齐、倒向稳定和严格停车逻辑接管。该设计避免平滑并入模块长期占用反馈降权，导致无法进入 1 cm 停车窗口。

## 12. 正倒车稳定、终点自然驶入与停车

### 12.1 倒向直线稳定

倒向、直线路径、非捕获状态下，横向或航向误差达到进入阈值时激活；误差同时降至退出阈值才关闭。激活后可使用独立反馈缩放、速度上限和反馈转角上限。

### 12.2 倒向虚拟收敛航向

普通倒向直线：

$$
e_{\psi,target}^{rev}=k_r\mathrm{atan2}(-e_{y,excess},L_r)
$$

终点附近改用终端切线横向误差，并令：

$$
L_r=\max(L_{floor},\max(0,e_s))
$$

前视下限避免最后厘米内直接指向目标点。

### 12.3 出弯恢复

当前曲率超过进入阈值后记录“处于弯道”；降至退出阈值时启动恢复距离。每周期按：

$$
D_{remain,k+1}=\max(0,D_{remain,k}-|v_{meas}|\Delta t)
$$

衰减。恢复状态默认不额外限速，但使用比普通直线更强的反馈缩放。

### 12.4 终点对齐

终点距离进入正/倒向对齐起始距离，且终端横向误差或路径航向误差超过激活阈值时，强化直线反馈。当前 `terminal_alignment_speed_limit_enable=false`，因此默认只增强反馈，不制造固定低速台阶。

### 12.5 自然终点速度

进入 `terminal_slowdown_distance` 后：

$$
v_{terminal}=k_s e_s
$$

当前正倒向均为 $k_s=0.5$，因此理想上：

| 纵向剩余距离 $e_s$ | 比例速度上限 |
|---:|---:|
| 2.0 m | 1.00 m/s |
| 1.0 m | 0.50 m/s |
| 0.5 m | 0.25 m/s |
| 0.2 m | 0.10 m/s |
| 0.1 m | 0.05 m/s |
| 0.02 m | 0.01 m/s |

若 $e_s\le0$，说明已经到达或越过终端切平面，继续保持原挡位只会增大纵向误差，因此速度直接为零。

### 12.6 终点修正速度地板

$$
v_{floor}=v_{min}+k_{fy}\max(0,|e_{t,y}|-e_{t,y}^{tol})
+k_{fd}\max(0,d_t-d_t^{tol})
$$

该地板被限制在 `terminal_correction_speed_max` 内，并且只在仍有纵向余量时使用。倒向在横向已进入窗口、欧氏距离略超差且仍有至少 2 mm 纵向余量时，可保持实车可控爬行速度。

### 12.7 正倒向终点辅助转角

$$
\Delta\phi_t=\mathrm{sgn}(e_{t,y})
\,k_t\max(0,|e_{t,y}|-d_b)
\,b_{entry}\,b_{fade}
$$

它在起始距离内渐入，并在停车窗口前淡出到零，避免最后形成弯钩或破坏航向。

### 12.8 单调速度包络

进入 `terminal_monotonic_release_distance` 后：

$$
v_{cap,k}=\min(v_{cap,k-1},v_{computed,k})
$$

定位、曲率或状态波动只能继续降低速度上限，不能再次提高。

### 12.9 停车与日志

三项位置条件同时满足后，控制器在当前周期：

1. 输出零速度；
2. 设置保持计数；
3. 按 `terminal_stop_hold_cycles` 决定是否立即锁存；
4. 发布诊断；
5. 只打印一次：

```text
FINAL BASE_LINK STOP: dist=..., required<=0.0100 m,
s=..., cte=..., actual_v=..., actual_w=..., direction=...
```

当前 `terminal_stop_hold_cycles=1`，因此首次满足窗口即锁存。

## 13. 完整参数表与调参方法

### 13.0 通用原则

1. **先标定底盘和参考点，再调 LQR。** 几何、外参或命令接口错误时，Q/R 无法从根本上解决问题。
2. **参数一次只改一组。** 建议每次改动 10%～20%，保留同一路径、同一速度和至少 3 次重复测试。
3. **所有长度阈值都要与定位噪声比较。** 1 cm 阈值不适合定位静止 P95 已超过 1 cm 的系统。
4. **数组参数必须同类型。** ROS 2 YAML 中 `[1, 0.92]` 会因整数/浮点混合解析失败，应写 `[1.0, 0.92]`。
5. **修改后确认运行时参数。** 源码目录 YAML、安装目录 YAML 和 launch 覆盖值可能不同。


### 13.1 插件注册

插件名称和 `pluginlib` 类型必须与 `plugin_description.xml`、`nav_core` 启动配置保持一致。一般更换底盘时不修改这一组。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `plugin_name` | `lqr` | — | 插件实例名称。参数通常以承载插件的 `nav_core` 节点为作用域声明。 | 保持为 `lqr`；只有同时修改启动文件、插件加载名称和诊断脚本时才改。 |
| `plugins` | `{controller: controller/lqr}` | — | 插件映射，当前将 `controller` 角色绑定到 `controller/lqr`。 | 保持与 `plugin_description.xml` 中 class name 一致。 |

### 13.2 底盘速度、几何与转向约束

这是更换底盘时最先标定的一组。几何参数错误会同时影响曲率前馈、DARE 输入矩阵、最大曲率和 `cmd_vel.angular.z`。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `target_linear_velocity` | `1.0` | m/s | 无其他速度约束时的巡航目标速度。 | 先设为安全测试速度；路径稳定后逐步提高。希望直线尽量使用 1 m/s 时设为 `1.0`。 |
| `min_linear_velocity` | `0.06` | m/s | 非终点区域的最低非零速度。防止普通跟踪进入底盘不可控低速区。 | 依据底盘最低可重复运动速度设置；过大将使小误差修正过冲，过小会出现有指令但车不动。 |
| `max_linear_velocity` | `1.0` | m/s | 控制器发布的绝对线速度上限。 | 不得超过底盘、场地和定位允许值；应不小于 `target_linear_velocity`，源码会强制不低于 `min_linear_velocity`。 |
| `max_angular_velocity` | `1.2` | rad/s | `cmd_vel.angular.z` 的绝对上限。 | 根据底盘命令接口和实测极限设置；过小会截断急弯，过大可能使适配器给出过激转向。 |
| `max_linear_acceleration`<br>`max_linear_deceleration` | `0.8`<br>`0.8` | m/s² | 速度平滑器每周期允许的加速和减速变化率。 | 加速过大易顿挫，过小恢复 1 m/s 太慢；减速过小会弯前或终点制动滞后。优先依据实车阶跃响应设置。 |
| `wheel_base` | `0.494` | m | 前后轴距 `L`。用于双阿克曼曲率关系、最大转角派生和 DARE 输入矩阵。 | 使用车辆几何定义中与控制模型一致的轴距；误差会直接表现为转弯半径偏差。 |
| `track_width` | `0.364` | m | 轮距 `W`。用于中央转角与内轮转角换算。 | 测量左右轮控制几何中心之间距离；不要用车体外宽。 |
| `max_steering_angle` | `0.698` | rad | 允许的最大内侧车轮转角。 | 使用底盘实际可达且安全的内轮角；过大不会提升实际能力，反而会导致模型与执行不一致。 |
| `min_turning_radius` | `0.59` | m | 允许的最小车辆轨迹转弯半径。参与最大可用转角和最大曲率派生。 | 以实车低速稳定转弯测试为准；设置过小会请求不可达曲率。 |
| `dual_ackermann_mode_radius_margin` | `0.03` | m | 在最小转弯半径上附加的安全余量。 | 转向接近机械极限或轮胎擦滑时增大；需要更充分利用转向范围时减小，但不应为负。 |

### 13.3 底盘静态执行模型

该组用于补偿“指令曲率正确，但底盘实际角速度幅值不足或方向响应不同”的确定性误差。补偿只作用于路径前馈，不直接放大 DARE-LQR 反馈。更换底盘时应重新录制开环 `cmd_vel → odom.twist` 数据。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `empirical_feedforward_compensation_enable` | `true` | bool | 启用角速度执行增益逆模型。 | 没有完成底盘辨识时先设 `false`；确认查表可靠后再开启。 |
| `empirical_yaw_rate_breakpoints`<br>`empirical_yaw_rate_gains` | `[0.0, 0.02, 0.05, 0.1, 0.15, 0.2]`<br>`[1.0, 0.92, 0.92, 0.89, 0.855, 0.82]` | rad/s；无量纲 | 角速度绝对值断点和对应的实际/指令稳态增益。源码线性插值。两个数组必须等长、断点递增且元素类型一致。 | 用开环稳态平台计算 `gain=\|ω_actual\|/\|ω_cmd\|`。ROS 2 YAML 数组必须全部写为浮点数，例如首项写 `0.0`、`1.0`，不能混用整数。 |
| `empirical_feedforward_compensation_strength` | `0.7` | 0～1 | 从不补偿到完整逆增益补偿的插值强度。补偿比例为 `1 + strength*(1/gain-1)`。 | 首次接入新底盘建议 `0.3～0.7`；弯道仍稳定偏外再增大，出现过转向则减小。 |
| `empirical_feedforward_compensation_max_scale` | `1.18` | 倍数 | 前馈曲率最大放大倍率。 | 用于限制辨识噪声或低增益点造成的过补偿；通常先限制在 `1.1～1.2`。 |
| `empirical_feedforward_compensation_min_yaw_rate` | `0.01` | rad/s | 名义角速度低于该值时不做查表补偿。 | 应高于静止角速度噪声和极小曲率区，避免直线附近放大噪声。 |
| `steering_delay_compensation_enable` | `true` | bool | 启用方向相关的转向延迟预瞄。 | 底盘存在明确、可重复的转向迟滞时开启。 |
| `positive_curvature_steering_delay`<br>`negative_curvature_steering_delay` | `0.18`<br>`0.09` | s | 两种物理转向方向的起效延迟。倒车时源码使用 `direction × motion_curvature` 选择物理方向。 | 由连续运动中的角速度阶跃辨识；不要用 publisher 重启造成的额外 DDS 延迟。入弯一侧偏晚则增加对应延迟。 |
| `steering_delay_compensation_strength` | `0.7` | 0～1 | 方向延迟转换为附加预瞄距离 `\|v\| τ strength` 的比例。 | 从 `0.5` 附近开始；过大可能过早入弯，过小仍会迟转。 |
| `steering_delay_max_preview_distance` | `0.18` | m | 延迟补偿附加预瞄距离上限。 | 不应大于局部路径能可靠表达的前方长度；短路径或急转角时应保守。 |
| `straight_steering_bias_enable` | `false` | bool | 启用近直线、低误差状态下的固定中央转角零偏补偿。 | 优先在底盘驱动层校零；只有开环直线存在可重复同向偏移且符号已验证时开启。 |
| `straight_steering_bias_central_angle` | `0.00147` | rad | 固定中央转角偏置。 | 由长直线净航向变化换算；若偏移更严重，先检查符号，必要时反号，不要盲目增大。 |
| `straight_steering_bias_activation_curvature` | `0.02` | 1/m | 只有名义前馈曲率小于该值时才允许零偏补偿。 | 应略小于直线/曲线分类阈值，避免在弯道中叠加固定偏置。 |
| `straight_steering_bias_max_lateral_error`<br>`straight_steering_bias_max_heading_error` | `0.05`<br>`0.08` | m；rad | 零偏补偿允许的最大横向和航向误差。误差越接近上限，补偿越淡出。 | 应只覆盖已基本稳定的直线状态；过大可能在路径捕获时产生错误固定转向。 |
| `straight_steering_bias_terminal_fade_distance` | `0.6` | m | 接近终点时将固定零偏淡出。 | 至少大于停车窗口；若终点朝向受偏置影响，增大该距离。 |

### 13.4 底盘动态、制动与方向差异

该组来自实车动态辨识。它决定曲率变化率、制动速度包络以及倒向专用稳定逻辑。方向对称、执行性能好的新底盘可关闭相应补偿。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `empirical_yaw_dynamics_enable` | `true` | bool | 使用实测横摆角加速度限制曲率变化率。 | 完成动态辨识后开启；未辨识时可关闭并仅使用 `max_curvature_rate`。 |
| `empirical_max_yaw_acceleration` | `0.6` | rad/s² | 普通转向时允许的最大横摆角加速度。 | 取连续控制测试中平滑且可重复的值，不应直接采用偶发峰值。减小更平稳，增大响应更快。 |
| `empirical_sign_change_max_yaw_acceleration` | `0.35` | rad/s² | 曲率正负号切换时更严格的横摆角加速度上限。 | S 弯或回正冲击明显时减小；换向过慢时适当增大。 |
| `empirical_terminal_max_yaw_acceleration`<br>`empirical_terminal_yaw_accel_distance` | `0.25`<br>`0.6` | rad/s²；m | 终点附近的横摆角加速度上限及其生效距离。 | 终点轮子摆动或低速左右修正明显时减小加速度或增大生效距离；收敛太慢则反向调整。 |
| `empirical_curvature_rate_speed_floor` | `0.1` | m/s | 由 `κ_dot_max=ω_dot_max/max(\|v\|, floor)` 换算时的速度下限。 | 防止低速时允许无限大的曲率变化率；一般取底盘可控低速量级。 |
| `empirical_braking_speed_limit_enable` | `true` | bool | 启用实测制动距离反解得到的速度上限。 | 有可靠主动零速度制动数据时开启。 |
| `empirical_braking_quadratic_coefficient`<br>`empirical_braking_linear_coefficient` | `0.3`<br>`0.085` | s²/m；s | 制动模型 `d=a v²+b v` 的系数，源码反解正根得到允许速度。 | 用主动持续发布零速度的实车数据拟合，不能用停止发布命令后的看门狗停车距离替代。 |
| `empirical_braking_safety_margin` | `0.0` | m | 从可用纵向距离中扣除的安全余量。 | 安全场景可取正值；厘米级停车中设置过大会在停车窗口外提前压到零，当前为 `0.0`。 |
| `empirical_braking_activation_distance` | `0.25` | m | 只有终点纵向距离不小于该值时才应用制动模型。 | 应把最后精细驶入区留给连续比例速度律；若末端提前停住，减小该值或安全余量。 |
| `reverse_straight_stabilization_enable` | `true` | bool | 启用倒向直线带滞回的稳定状态机。 | 倒向与正向性能对称时可关闭；倒向存在可重复直线偏置时开启。 |
| `reverse_straight_feedback_scale` | `0.65` | 倍数 | 倒向稳定状态激活时的 LQR 反馈缩放。 | 倒向直线收敛不足时增大；出现 S 型反复穿越时减小并检查虚拟收敛航向。 |
| `reverse_straight_speed_limit` | `1.0` | m/s | 倒向稳定状态的速度上限。当前 `1.0` 等效不额外限速。 | 短直线无法收敛时可降低；希望尽量保持巡航速度时维持为最大速度。 |
| `reverse_straight_lateral_enter`<br>`reverse_straight_lateral_exit` | `0.01`<br>`0.005` | m | 倒向稳定器横向误差进入/退出阈值，形成滞回。 | 进入阈值应大于定位噪声，退出阈值应更小；状态频繁切换时扩大两者间隔。 |
| `reverse_straight_heading_enter`<br>`reverse_straight_heading_exit` | `0.02`<br>`0.008` | rad | 倒向稳定器航向误差进入/退出阈值。 | 与横向阈值共同设置；航向残差长期抵消横向修正时降低进入阈值。 |
| `reverse_straight_feedback_max_central_steering` | `0.04` | rad | 倒向稳定或终点对齐时直线 LQR 反馈中央转角上限。 | 偏置收敛慢时适当增大，S 弯或轮子摆动时减小。 |
| `reverse_convergence_heading_enable` | `true` | bool | 启用倒向虚拟收敛航向，允许车辆保持小角度逐渐消除横向偏差。 | 倒向直线存在稳定横向偏置时开启；方向对称底盘可关闭。 |
| `reverse_convergence_heading_gain` | `0.55` | 倍数 | 虚拟航向 `gain*atan2(-e_y, lookahead)` 的增益。 | 横向误差消除慢时增大；出现穿越路径、S 型回摆时减小。 |
| `reverse_convergence_heading_max` | `0.025` | rad | 虚拟收敛航向绝对上限。 | 限制倒向并线角度；过大易弯钩，过小收敛距离不足。 |
| `reverse_convergence_heading_deadband` | `0.004` | m | 生成倒向虚拟航向前扣除的横向误差死区。 | 略高于定位随机噪声；过小会持续微调，过大保留稳态偏置。 |
| `reverse_convergence_straight_lookahead` | `1.0` | m | 普通倒向直线虚拟航向的收敛前视长度。 | 增大使并入更缓、更少 S 型；减小使收敛更快但更激进。 |
| `reverse_convergence_terminal_start_distance`<br>`reverse_convergence_terminal_lookahead_floor` | `1.6`<br>`0.45` | m | 倒向终点附近改用终端切线 CTE，并限制最小收敛前视长度。 | 开始距离应覆盖出弯后短直线；前视下限防止最后厘米内直接指向终点产生弯钩。 |
| `reverse_terminal_longitudinal_speed_gain` | `0.5` | s⁻¹ | 倒向终点比例速度上限 `v=k_s*s`。 | 当前与正向同为 `0.5`。过大可能越过，过小末段拖沓。 |
| `reverse_terminal_correction_min_velocity` | `0.012` | m/s | 倒向尚未满足停车窗口时的最低可控修正速度。 | 依据倒盘最低可重复速度设置；过高会越过，过低会原地不动。 |
| `reverse_terminal_crawl_min_along_distance`<br>`reverse_terminal_crawl_max_lateral_error` | `0.002`<br>`0.01` | m | 启用倒向可控爬行所需的最小纵向余量和最大横向误差。 | 只允许在横向已基本合格且仍有纵向余量时爬行；扩大条件会增加越过风险。 |

### 13.5 误差、状态与精度阈值

这一组与定位噪声、路径质量和目标精度直接相关。停车阈值均按 `base_link → base_link` 计算。路径捕获只负责普通路径并入；进入终点单调减速区后源码会强制交接给终点控制。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `lateral_error_limit`<br>`heading_error_limit` | `0.25`<br>`0.7` | m；rad | 送入 LQR 的横向和航向误差限幅。 | 应覆盖正常可恢复范围；过大使异常误差产生饱和转向，过小会降低大偏差恢复能力。 |
| `lateral_error_deadband`<br>`heading_error_deadband` | `0.0015`<br>`0.002` | m；rad | 最近线段误差的零死区。 | 略高于定位与路径离散噪声；直线微摆时增大，保留偏差时减小。 |
| `path_acquisition_speed_limit_enable` | `true` | bool | 启用初始路径捕获、重新捕获和捕获限速。 | 起点可能偏离路径时建议开启。 |
| `path_acquisition_enter_lateral_error`<br>`path_acquisition_enter_heading_error` | `0.05`<br>`0.1` | m；rad | 首次任务进入路径捕获的横向/航向阈值。 | 正常起始误差不应频繁触发；过小会在厘米级误差时降速，过大可能高速并入。 |
| `path_acquisition_exit_lateral_error`<br>`path_acquisition_exit_heading_error` | `0.012`<br>`0.025` | m；rad | 退出路径捕获所需误差阈值。 | 应小于进入阈值且略高于定位噪声；过严会长期保持捕获状态。 |
| `path_acquisition_full_lateral_error`<br>`path_acquisition_full_heading_error` | `0.25`<br>`0.4` | m；rad | 捕获严重程度归一化到 1 的误差上限。 | 决定速度从 `max` 向 `min` 下降的斜率；值越大，同一误差下限速越弱。 |
| `path_reacquisition_enter_lateral_error`<br>`path_reacquisition_enter_heading_error` | `0.08`<br>`0.18` | m；rad | 已经稳定跟踪后重新进入捕获的阈值。 | 应高于首次进入阈值，避免正常波动触发；路径失控后恢复太晚则减小。 |
| `path_acquisition_min_speed`<br>`path_acquisition_max_speed` | `0.25`<br>`0.7` | m/s | 捕获状态按误差严重程度插值的最低/最高速度。 | 大偏差穿越路径时降低最大值；并入过慢时提高，但不应超过底盘在当前误差下可控速度。 |
| `path_acquisition_stable_cycles` | `5` | 周期 | 连续满足退出阈值多少个周期后关闭捕获。 | 定位抖动大时增加；切换拖延时减小。当前控制周期 0.1 s，5 周期约 0.5 s。 |
| `path_acquisition_convergence_heading_enable` | `true` | bool | 启用捕获阶段的渐近并入目标航向。 | 位置和朝向同时偏离、直接 LQR 易走 S 型时开启。 |
| `path_acquisition_convergence_heading_gain` | `0.8` | 倍数 | 捕获目标航向 `gain*atan2(-e_y, lookahead)` 的增益。 | 并入慢时增大，穿越路径后反向回摆时减小。 |
| `path_acquisition_convergence_heading_max` | `0.14` | rad | 捕获目标航向绝对上限。 | 决定最大切入角；场地狭窄或 S 型明显时减小。 |
| `path_acquisition_convergence_heading_deadband` | `0.008` | m | 生成捕获目标航向前扣除的横向死区。 | 略高于定位噪声；过小会持续偏转，过大退出后保留偏差。 |
| `path_acquisition_convergence_lookahead` | `1.2` | m | 捕获渐近航向的等效并入长度。 | 增大更平滑、占用距离更长；减小并入更快、更易过冲。 |
| `path_acquisition_convergence_max_reference_curvature` | `0.05` | 1/m | 只有参考路径足够直时才生成渐近并入航向。 | 避免在急弯中使用直线并入模型；弯道并入需要更强支持时可适当提高。 |
| `path_acquisition_lateral_feedback_scale` | `0.45` | 0～1 | 捕获期间送入 LQR 的横向误差缩放。 | S 型穿越时减小；并入不足时增大。与目标航向增益联调。 |
| `path_acquisition_feedback_scale` | `0.75` | 0～1 | 捕获期间整个 LQR 反馈量缩放。 | 转向过猛时减小，恢复过慢时增大。 |
| `path_acquisition_max_central_steering` | `0.12` | rad | 捕获期间 LQR 反馈中央转角上限。 | 限制大偏差时的激进转向；过小无法并入，过大易穿越路径。 |
| `terminal_stop_distance`<br>`terminal_stop_s_tolerance`<br>`terminal_stop_lateral_tolerance` | `0.01`<br>`0.01`<br>`0.01` | m | 停车欧氏距离、终端切线纵向误差和横向误差阈值。三项同时满足才停车。 | 当前均为 1 cm。阈值不得明显小于定位重复性；改变精度时三项应协同设置。 |
| `terminal_stop_exit_distance` | `0.03` | m | 已进入停车保持后允许的距离滞回范围。 | 只用于保持，不是首次停车阈值。定位抖动导致反复启动时增大。 |
| `terminal_stop_hold_cycles` | `1` | 周期 | 首次满足停车窗口后的锁存确认周期数。当前为 `1`，同周期锁存。 | 需要更严格时间稳定性时增加，但会延迟进入 stop；不可把它误认为位置阈值。 |
| `terminal_alignment_lateral_activation`<br>`terminal_alignment_heading_activation` | `0.008`<br>`0.015` | m；rad | 终点对齐状态的横向或航向激活阈值。 | 过小会在末端频繁强化反馈，过大可能带偏差进入停车区。 |
| `forward_terminal_lateral_convergence_deadband`<br>`reverse_terminal_lateral_convergence_deadband` | `0.004`<br>`0.003` | m | 正向/倒向终点辅助横向修正死区。 | 根据定位噪声和方向差异设置；过小导致末段轮子微摆。 |
| `straight_output_curvature_deadband`<br>`straight_output_deadband_lateral_error`<br>`straight_output_deadband_heading_error` | `0.006`<br>`0.006`<br>`0.012` | 1/m；m；rad | 直线已稳定时将极小输出曲率压到零的联合条件。 | 直线微摆时适当增大；存在固定偏差或小修正被吞掉时减小。 |
| `straight_sign_reversal_min_curvature` | `0.025` | 1/m | 直线中目标曲率反号且幅值低于该值时先回到零，抑制小幅左右反复切换。 | S 型微摆时增大；必要反向修正被阻断时减小。倒向稳定状态会放行必要修正。 |

### 13.6 四状态 DARE-LQR 核心参数

这一组决定反馈控制器本身。应在几何、前馈、延迟、速度规划和参考点都正确后再调。单独比较 Q、R 数值没有意义，应同时观察在线增益 `K0～K3`、曲率输出和闭环误差。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `lqr_q_lateral` | `12.0` | 代价权重 | 横向误差 `e_y` 权重。 | 增大可加快位置回归，但过大会产生穿越路径和直线摆动。 |
| `lqr_q_lateral_rate` | `0.3` | 代价权重 | 横向误差变化率权重。 | 增大可抑制误差变化，但变化率含定位差分噪声，过大会使控制敏感。 |
| `lqr_q_heading` | `10.0` | 代价权重 | 航向误差 `e_ψ` 权重。 | 入弯/出弯航向滞后时可增大；过大会使车辆过早回正或与横向项抵消。 |
| `lqr_q_heading_rate` | `0.25` | 代价权重 | 航向误差变化率权重。 | 用于抑制航向动态；过大可能放大角度差分噪声。 |
| `lqr_r_steering` | `4.2` | 代价权重 | 中央转角修正输入代价。 | 增大更平滑但收敛慢；减小反馈更强但更易振荡。通常以 10%～20% 小步调整。 |
| `lqr_feedback_scale` | `1.0` | 倍数 | DARE 输出后的全局反馈缩放。 | 用于整体微调，不应代替 Q/R 或底盘前馈补偿。 |
| `lqr_error_rate_filter_alpha` | `0.25` | 0～1 | 误差变化率一阶低通中当前样本权重。 | 增大响应快、噪声大；减小更平滑、相位滞后更大。 |
| `straight_reference_curvature_threshold` | `0.025` | 1/m | 当前曲率和前馈曲率都低于该值时视为直线。 | 应高于路径离散造成的直线曲率噪声、低于真实弯道曲率。 |
| `straight_feedback_scale`<br>`curve_feedback_scale` | `0.55`<br>`1.18` | 倍数 | 直线和曲线状态下的反馈缩放。 | 直线 S 型时降低 `straight`；弯道偏外且前馈正确时提高 `curve`。不要同时大幅修改。 |
| `straight_feedback_max_central_steering` | `0.085` | rad | 普通直线状态的 LQR 反馈中央转角上限。 | 限制直线大幅反打；偏差无法恢复时增大，S 型时减小。 |

### 13.7 路径参考、短预瞄与曲率前馈

短预瞄直接改变路径曲率前馈和 DARE 工作点，普通前瞻点主要用于参考路径准备与可视化。路径误差仍由车辆原点到最近线段投影计算。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `path_curvature_feedforward_gain` | `1.0` | 倍数 | 路径预瞄曲率的名义前馈增益。 | 几何和实车执行补偿正确后应接近 1；弯道整体稳定偏外可小幅增大，过转则减小。 |
| `path_curvature_window_distance` | `0.18` | m | 三点外接圆曲率估计的弧长窗口。源码最小实际跨度为 0.12 m。 | 路径噪声大时增大，急弯被过度平滑时减小；应大于若干路径采样间距。 |
| `curve_preview_enable` | `true` | bool | 启用未来曲率短预瞄。 | 需要提前入弯和提前回正时开启。 |
| `curve_preview_min_distance`<br>`curve_preview_speed_gain`<br>`curve_preview_max_distance` | `0.08`<br>`0.25`<br>`0.38` | m；s；m | 短预瞄距离 `clip(min + gain*\|v\|, min, max)`。 | 入弯偏晚可增大；过早转弯或短直线被提前弯道影响时减小，优先调 `max` 和 `gain`。 |
| `curve_preview_samples` | `7` | 个 | 短预瞄区间内的未来曲率采样数。 | 增加可平滑但提高计算量；路径稀疏时无需过多。 |
| `curve_preview_blend` | `0.5` | 0～1 | 当前曲率与未来平均曲率的最大融合比例。 | 入弯偏晚时增大，直线提前转向或出弯过早时减小。 |
| `curve_preview_activation_curvature` | `0.03` | 1/m | 短预瞄和方向延迟补偿的曲率激活阈值。 | 应高于直线路径曲率噪声；过低会在直线上误判弯道。 |
| `lookahead_min_distance`<br>`lookahead_speed_gain`<br>`lookahead_max_distance` | `0.3`<br>`0.42`<br>`0.85` | m；s；m | 普通前瞻距离 `clip(min + gain*\|v\|, min, max)`。 | 主要影响可视化和路径延长尺度，不直接替代最近线段误差。短路径参考不足时可适当增大最小长度。 |
| `direction_filter_margin` | `0.08` | m | 运动坐标系中允许保留的车辆后方路径长度。 | 后方路径干扰最近点时减小；路径在车辆附近截断严重时适当增大。 |
| `minimum_reference_length` | `0.4` | m | 过滤后参考路径不足时沿终端切线延长到的最小长度。 | 应至少覆盖低速前瞻；过长可能在短路径末端引入虚拟直线影响。 |
| `terminal_heading_lookback_distance` | `0.65` | m | 没有可靠外部终点航向时，从路径末端回看该距离计算终端切线。 | 末端点噪声大时增大，短末段曲率变化需要保留时减小。 |

### 13.8 弯道速度规划与出弯恢复

长预瞄只计算前方弯道速度上限；当前曲率限速和误差限速在本周期直接生效。出弯恢复状态按实车已行驶距离衰减。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `curve_anticipation_enable` | `true` | bool | 启用长预瞄弯前减速。 | 高速运行或急弯路径建议开启。 |
| `curve_anticipation_min_distance`<br>`curve_anticipation_speed_gain`<br>`curve_anticipation_max_distance` | `0.9`<br>`1.5`<br>`2.5` | m；s；m | 长预瞄距离 `clip(min + gain*\|v\|, min, max)`。 | 弯前减速过晚时增大，直线过早减速时减小。 |
| `curve_anticipation_samples` | `20` | 区间数 | 长预瞄扫描区间数，源码实际评估 `samples+1` 个位置。 | 增加提高曲率峰值发现概率但增加计算量。 |
| `curve_anticipation_curvature_threshold` | `0.05` | 1/m | 长预瞄认为前方进入弯道的曲率阈值。 | 直线误减速时提高，弯道漏检时降低。 |
| `curve_anticipation_lateral_acceleration` | `0.1` | m/s² | 前方弯道目标速度的横向加速度上限。 | 减小会更早/更慢过弯，提高稳定性；过小会长期低速。 |
| `curve_anticipation_min_speed` | `0.24` | m/s | 长预瞄弯道速度下限。 | 低于底盘稳定运动速度不可取；急弯仍偏外时可适当降低。 |
| `curve_anticipation_deceleration` | `0.65` | m/s² | 根据距弯道入口的距离反推允许速度时使用的减速度。 | 值大表示允许更晚减速；弯前制动太晚则减小。 |
| `curve_anticipation_safety_distance` | `0.25` | m | 从弯道入口距离中扣除的安全距离。 | 增大使减速更早；直线提前降速时减小。 |
| `curvature_speed_limit_enable` | `true` | bool | 按当前目标曲率和横向加速度限制速度。 | 建议开启。 |
| `max_lateral_acceleration` | `0.32` | m/s² | 当前曲率限速 `v<=sqrt(a_y/\|κ\|)` 使用的横向加速度上限。 | 弯道偏外或侧滑时减小；弯道过慢且跟踪稳定时增大。 |
| `error_speed_limit_enable` | `true` | bool | 按横向/航向误差比例降低速度。 | 大误差仍高速运行时开启。 |
| `lateral_error_speed_gain`<br>`heading_error_speed_gain` | `0.55`<br>`0.38` | 1/m；1/rad | 误差速度比例 `1/(1+k_y\|e_y\|+k_ψ\|e_ψ\|)` 的增益。 | 误差大时仍太快则增大；小误差导致频繁降速则减小。 |
| `speed_curvature_lateral_error_gain`<br>`speed_curvature_heading_error_gain` | `0.36`<br>`0.66` | 1/m²；1/(m·rad) | 构造速度限幅用目标曲率估计 `κ_ff + k_y e_y + k_ψ e_ψ`。不直接改变转向。 | 误差修正需要大转向却未降速时增大；直线误差很小仍被降速时减小。 |
| `post_curve_recovery_enable` | `true` | bool | 检测弯道结束后进入按距离计的恢复状态。 | 出弯后短直线仍有残余误差时开启。 |
| `post_curve_enter_curvature`<br>`post_curve_exit_curvature` | `0.06`<br>`0.025` | 1/m | 进入“曾经在弯道”与判定出弯的曲率阈值。 | `enter` 应大于 `exit` 形成滞回；路径曲率噪声导致误触发时提高。 |
| `post_curve_recovery_distance` | `2.4` | m | 出弯后恢复状态持续的行驶距离。 | 短直线末端仍有残余误差时增大；长期影响正常直线时减小。 |
| `post_curve_recovery_speed_limit` | `1.0` | m/s | 出弯恢复状态速度上限。当前 `1.0` 不额外降速。 | 需要给短直线更多收敛时间时降低；希望尽量保持 1 m/s 时保持最大值。 |
| `post_curve_feedback_scale` | `0.85` | 倍数 | 出弯恢复状态的反馈缩放。当前高于普通直线 `0.55`。 | 残余误差消除慢时增大；出弯后 S 型修正时减小。 |

### 13.9 终点自然驶入、对齐与停车

终点速度首先由终端切线纵向剩余量决定，并与制动包络、低速可控速度和单调速度上限组合。当前停车位置条件不包含独立航向阈值；最终航向由末端路径切线和运动过程中的反馈保证。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `terminal_stop_on_path_end_enable` | `true` | bool | 没有外部有效终点时，是否使用当前路径末点作为停车目标。 | 路径末点就是任务终点时开启；局部路径会滚动截断时应关闭或确保有外部终点。 |
| `terminal_stop_latch_enable` | `true` | bool | 满足停车窗口后是否锁存零速度。 | 防止定位波动重新启动，建议开启。 |
| `terminal_slowdown_enable` | `true` | bool | 启用终点比例减速和停车判定。 | 需要自然驶入停车时开启。 |
| `terminal_slowdown_distance` | `2.0` | m | 进入终点纵向比例减速逻辑的欧氏距离。 | 与 `v=k_s*s` 配合。希望更久保持 1 m/s 可减小，但必须留足制动和横向收敛距离。 |
| `terminal_min_linear_velocity` | `0.01` | m/s | 终点模型速度下限及修正速度基准。DARE 模型实际还强制不低于 0.05 m/s。 | 设置为底盘能稳定执行的低速量级；过高易越过，过低可能不动。 |
| `terminal_longitudinal_speed_gain` | `0.5` | s⁻¹ | 正向终点比例速度上限 `v=k_s*s`。 | 当前 0.5 对应 2 m 处 1 m/s。过大减速晚，过小末段拖沓。 |
| `terminal_correction_min_velocity` | `0.01` | m/s | 正向未满足停车窗口时最低可控修正速度。 | 依据底盘低速响应设置；需与控制周期共同评估每周期位移。 |
| `terminal_correction_speed_max` | `0.03` | m/s | 由横向/距离误差计算的修正速度地板上限。 | 过大易冲过，过小无法克服底盘死区。 |
| `terminal_lateral_error_speed_floor_gain`<br>`terminal_distance_error_speed_floor_gain` | `1.2`<br>`0.2` | s⁻¹ | 终点横向误差超差和欧氏距离超差对修正速度地板的增益。 | 末段停滞时小幅提高；发生突然加速或过冲时降低。速度地板最终仍受纵向安全条件约束。 |
| `terminal_alignment_enable` | `true` | bool | 启用最后直线的反馈强化判定。 | 转弯后短直线需要在终点前消除残差时开启。 |
| `terminal_alignment_speed_limit_enable` | `false` | bool | 是否同时启用终点对齐专用速度上限。当前关闭，因此只改变反馈缩放。 | 需要更慢对齐时开启；自然速度优先时保持关闭。 |
| `terminal_alignment_start_distance`<br>`reverse_terminal_alignment_start_distance` | `1.2`<br>`1.6` | m | 正向/倒向终点对齐开始距离。 | 应覆盖出弯后的可用直线；倒向通常需要更早。 |
| `terminal_alignment_max_speed`<br>`terminal_alignment_min_speed`<br>`reverse_terminal_alignment_max_speed`<br>`reverse_terminal_alignment_min_speed` | `1.0`<br>`1.0`<br>`1.0`<br>`1.0` | m/s | 对齐速度上限开启时按纵向剩余距离插值的正/倒向最大和最小速度。当前均为 1.0，且开关关闭。 | 开启限速后，最大值决定刚进入对齐时速度，最小值决定接近停车窗口时速度；必须避免硬速度台阶。 |
| `terminal_alignment_feedback_scale`<br>`reverse_terminal_alignment_feedback_scale` | `0.95`<br>`0.9` | 倍数 | 正向/倒向终点对齐激活时的 LQR 反馈缩放。 | 终点横向误差消除慢时提高；末段 S 型或弯钩时降低。 |
| `forward_terminal_lateral_convergence_enable` | `true` | bool | 启用正向终点小幅横向辅助转角。 | 正向已经自然进入且无固定偏差时可关闭；当前用于提高 1 cm 重复性。 |
| `forward_terminal_lateral_convergence_start_distance`<br>`forward_terminal_lateral_convergence_fade_distance` | `1.2`<br>`0.03` | m | 正向辅助修正开始距离和最终淡出距离。 | 开始过晚无法收敛，过早影响普通跟踪；淡出距离必须大于停车距离并保证停车前归零。 |
| `forward_terminal_lateral_convergence_gain` | `1.0` | rad/m | 正向终点横向超差转换为辅助中央转角的增益。 | 偏差残留时小幅提高；末端弯钩或朝向变差时降低。 |
| `forward_terminal_lateral_convergence_max_steering` | `0.025` | rad | 正向辅助中央转角上限。 | 保持小值，避免变成“直接指向目标点”的停车控制。 |
| `reverse_terminal_lateral_convergence_enable` | `true` | bool | 启用倒向终点小幅横向辅助转角。 | 倒向停车重复性不足时开启。 |
| `reverse_terminal_lateral_convergence_start_distance`<br>`reverse_terminal_lateral_convergence_fade_distance` | `1.6`<br>`0.012` | m | 倒向辅助修正开始距离和最终淡出距离。 | 倒向通常需更早介入；淡出必须在停车窗口前完成。 |
| `reverse_terminal_lateral_convergence_gain` | `1.05` | rad/m | 倒向终点横向超差转换增益。 | 倒向 CTE 残留时提高，走 S 弯或越过时降低。 |
| `reverse_terminal_lateral_convergence_max_steering` | `0.028` | rad | 倒向辅助中央转角上限。 | 限制低速终点转向，过大易使前后停点左右不一致。 |
| `terminal_monotonic_speed_enable` | `true` | bool | 进入最后阶段后速度上限只允许下降，不允许因定位或预瞄波动再次升高。 | 需要避免“减速—加速—再减速”时开启。 |
| `terminal_monotonic_release_distance` | `2.2` | m | 单调速度区起始距离，同时是路径捕获向终点控制强制交接距离。 | 应不小于 `terminal_slowdown_distance`；过大过早禁止恢复速度，过小捕获逻辑可能侵入停车阶段。 |

### 13.10 曲率、转向与速度指令平滑

控制器先平滑曲率，再由双阿克曼几何生成角速度；线速度平滑后按 `angular.z/|linear.x|` 保持转向比值，不独立滤波角速度。

| 参数 | 当前默认值 | 单位 | 含义与作用位置 | 设置与调参方法 |
|---|---:|---|---|---|
| `curvature_smoothing_enable` | `true` | bool | 启用曲率变化率限制和一阶平滑。 | 实车使用建议开启。 |
| `curvature_smoothing_alpha` | `0.55` | 0～1 | 普通状态曲率平滑中当前目标权重。 | 增大响应快，减小更平滑。 |
| `curve_entry_smoothing_alpha` | `0.85` | 0～1 | 入弯或显著增大转向时的当前目标权重。 | 入弯偏晚时增大；冲击明显时减小。 |
| `curve_exit_smoothing_alpha` | `0.82` | 0～1 | 减小转向、出弯回正时的当前目标权重。 | 回正偏晚时增大；回正过快产生反摆时减小。 |
| `curve_entry_rate_multiplier` | `1.4` | 倍数 | 入弯时基础曲率变化率的倍率。 | 入弯不足时增大，机械冲击或过转时减小。 |
| `max_curvature_rate` | `3.0` | 1/(m·s) | 不考虑实测横摆动态时的基础最大曲率变化率。 | 与控制周期共同决定每周期最大变化；过大易冲击，过小转向滞后。 |
| `straight_sign_change_smoothing_alpha` | `0.3` | 0～1 | 直线附近曲率反号时的平滑系数。 | S 型微摆时减小；反向修正太慢时增大。 |
| `straight_sign_change_rate_multiplier` | `0.45` | 倍数 | 直线反号时曲率变化率倍率。 | S 型时减小，必要换向迟缓时增大。 |
| `command_smoothing_enable` | `true` | bool | 启用线速度加减速约束和一阶平滑。 | 实车建议开启。 |
| `command_smoothing_alpha` | `0.65` | 0～1 | 加速及一般速度变化时当前目标权重。 | 增大跟随快，减小更平滑但恢复巡航慢。 |
| `command_deceleration_alpha` | `0.65` | 0～1 | 减速时当前目标权重，源码取其与普通 alpha 的较大值。 | 弯前/终点减速滞后时增大；突然减速明显时减小，但需保证不越过。 |
| `control_period` | `0.1` | s | DARE 离散周期、状态差分周期、曲率/速度每周期变化量的时间基准。 | 必须与 `nav_core` 实际控制回调周期一致。周期不一致会同时破坏 K、导数、加速度和状态机计时。 |

## 14. 诊断话题与日志

源码使用 ROS 2 私有话题：

```text
~/lookahead_point
~/tracking_error
```

实际完整名称取决于承载插件的节点和命名空间：

```bash
ros2 topic list | grep -E 'lookahead_point|tracking_error'
```

### 14.1 前瞻点

- 类型：`geometry_msgs/msg/PoseStamped`；
- 内容：普通前瞻点；
- 发布前从统一运动坐标系转换回实际 `base_link` 方向。

### 14.2 `tracking_error` 数组

类型：`std_msgs/msg/Float64MultiArray`，当前固定 38 个元素。

| 索引 | 字段 | 单位/含义 |
|---:|---|---|
| 0 | `lateral_error` | m，最近线段横向误差 |
| 1 | `heading_error` | rad，最近线段航向误差 |
| 2 | `terminal_distance` | m，`base_link` 终点欧氏距离 |
| 3 | `commanded_linear_velocity` | m/s，最终线速度指令 |
| 4 | `commanded_yaw_rate` | rad/s，最终角速度指令 |
| 5 | `K0` | 横向误差反馈增益 |
| 6 | `K1` | 横向误差变化率反馈增益 |
| 7 | `K2` | 航向误差反馈增益 |
| 8 | `K3` | 航向误差变化率反馈增益 |
| 9 | `lookahead_distance` | m，普通前瞻距离 |
| 10 | `direction_sign` | 正向 `+1`，倒向 `-1` |
| 11 | `terminal_x` | m，终点在运动坐标系中的 x |
| 12 | `terminal_y` | m，终点在运动坐标系中的 y |
| 13 | `terminal_heading` | rad，终端切线 |
| 14 | `reverse_terminal_trim` | rad，倒向终点辅助中央转角 |
| 15 | `terminal_is_real_target` | 1/0，有效停车目标标志 |
| 16 | `terminal_along_error` | m，终端纵向误差 $e_s$ |
| 17 | `terminal_cross_track_error` | m，终端横向误差 $e_{t,y}$ |
| 18 | `terminal_stop_latched` | 1/0，停车锁存状态 |
| 19 | `current_path_curvature` | 1/m，当前位置曲率 |
| 20 | `preview_path_curvature` | 1/m，短预瞄融合曲率 |
| 21 | `feedforward_curvature` | 1/m，底盘补偿后前馈曲率 |
| 22 | `raw_commanded_curvature` | 1/m，平滑前最终目标曲率 |
| 23 | `smoothed_curvature` | 1/m，平滑后曲率 |
| 24 | `nominal_feedforward_curvature` | 1/m，补偿前名义前馈曲率 |
| 25 | `feedforward_execution_gain` | 实测角速度执行增益 |
| 26 | `feedforward_compensation_scale` | 实际前馈补偿倍率 |
| 27 | `steering_delay_preview_distance` | m，方向延迟附加预瞄距离 |
| 28 | `straight_bias_central_steering` | rad，直线固定偏置 |
| 29 | `forward_terminal_trim` | rad，正向终点辅助中央转角 |
| 30 | `empirical_curvature_rate_limit` | 1/(m·s)，当前实测动态曲率变化率上限 |
| 31 | `post_curve_recovery_remaining` | m，出弯恢复剩余距离 |
| 32 | `terminal_alignment_speed_limit` | m/s，终点对齐计算出的速度上限（即使开关关闭也用于诊断） |
| 33 | `reverse_convergence_heading_target` | rad，倒向虚拟收敛航向 |
| 34 | `reverse_straight_speed_limit` | m/s，倒向直线状态速度上限 |
| 35 | `reverse_straight_stabilization_active` | 1/0，倒向稳定状态 |
| 36 | `terminal_monotonic_speed_cap` | m/s，终点单调速度上限 |
| 37 | `path_acquisition_heading_target` | rad，路径捕获渐近目标航向 |

> 数组没有字段名的编译期约束。修改源码索引时必须同步数据分析脚本和本表；长期接口建议改为专用消息类型。

## 15. 推荐调试流程

### 15.1 更换底盘

1. 关闭所有闭环控制，仅发布 `cmd_vel`；
2. 标定 `wheel_base、track_width、max_steering_angle、min_turning_radius`；
3. 测试稳态线速度增益和最低可控速度；
4. 测试角速度稳态增益，建立断点表；
5. 测试正负物理转向方向延迟；
6. 测试角速度阶跃和回正，设置横摆动态限制；
7. 测试主动零速度制动距离，拟合 `d=a v²+b v`；
8. 先关闭方向专用补偿，测试正倒车对称性；
9. 只有存在可重复方向差异时，再启用倒向稳定/虚拟航向/终点辅助项；
10. 最后才调 Q、R 和反馈缩放。

### 15.2 DARE-LQR 调参

推荐固定路径和速度，按顺序进行：

1. `path_curvature_feedforward_gain`：让稳态圆弯半径基本正确；
2. `curve_preview_*` 和方向延迟：修正入弯/出弯时机；
3. `lqr_r_steering`：先确定整体平滑程度；
4. `lqr_q_heading`：修正航向跟随；
5. `lqr_q_lateral`：修正横向收敛；
6. 变化率权重和滤波：处理动态和噪声；
7. `straight_feedback_scale / curve_feedback_scale`：做场景分段微调；
8. 曲率平滑：最后处理机械冲击和直线微摆。

### 15.3 每次测试建议记录

```text
/cmd_vel
/odom
/lio/odom
/tf
/tf_static
实际 tracking_error 话题
```

至少统计：

- 横向误差 RMS / P95 / 最大值；
- 航向误差 RMS / P95；
- 弯道入口和出口时刻；
- 曲率正负号切换次数；
- 线速度单调性；
- 正倒向终点 `dist、s、cte`；
- 是否出现路径捕获长期不退出；
- 是否打印一次 `FINAL BASE_LINK STOP`。

## 16. 常见问题定位

| 现象 | 优先检查与调整 |
|---|---|
| 直线误差很小但速度只有约 0.5 m/s | 查看日志 `acquisition=true`；检查捕获进入/退出阈值和 `path_acquisition_max_speed`，不要误认为提前检测弯道。 |
| 初始位置和朝向偏差时走 S 型 | 增大捕获并入前视，减小捕获目标航向增益、横向反馈缩放或最大转角；确认速度上限不过高。 |
| 平滑并入后无法停车 | 确认最后 `terminal_monotonic_release_distance` 内 `acquisition=false`；检查终点交接代码和运行的二进制版本。 |
| 入弯偏晚、弯道偏外 | 先检查曲率、轴距和执行增益；再增大短预瞄/延迟补偿或曲线反馈，不要先大幅降低 R。 |
| 直线左右微摆 | 检查路径曲率噪声、外参和控制周期；适当增大死区、减小直线反馈或提高 R。 |
| 出弯后短直线无法回正 | 增大恢复距离或恢复反馈，必要时降低恢复速度上限；检查短预瞄是否能看到后续直线。 |
| 倒向直线稳定偏离 | 检查运动坐标系符号、倒向稳定状态、虚拟收敛航向和最小可控速度。 |
| 正倒向同一目标停点不同 | 确认目标是同一个 `base_link` 位置、外参只应用一次；再比较正倒向辅助转角、速度地板和路径末端切线。 |
| 最后减速后又加速 | 检查 `terminal_monotonic_speed_enable` 和诊断索引 36；进入最后阶段后该值应只下降。 |
| 终点前提前零速 | 检查 `e_s` 是否已小于等于零、制动模型安全余量和激活距离、最低速度是否被底盘死区吞掉。 |
| 满足位置但很久才进入 stop | 检查 `terminal_stop_hold_cycles` 和上层 `nav_core` 的停车状态条件；当前 LQR 默认为 1 周期立即锁存。 |
| YAML 报 `Sequence should be of same type` | 数组全部使用同类型浮点数，如 `[1.0, 0.92]`，不能写 `[1, 0.92]`。 |
| 修改 YAML 无效果 | 删除对应 `build/install`、重新编译并 `source install/setup.bash`；再用 `ros2 param get` 验证运行值。 |

## 17. 编译、运行与参数检查

### 17.1 编译

```bash
cd ~/neu_nav_ws
source /opt/ros/humble/setup.bash

rm -rf build/lqr install/lqr

colcon build \
  --packages-select lqr \
  --symlink-install

source install/setup.bash
```

若 `nav_core` 接口同时修改：

```bash
colcon build --packages-up-to lqr --symlink-install
source install/setup.bash
```

### 17.2 检查安装文件

```bash
ros2 pkg prefix lqr
ls "$(ros2 pkg prefix lqr)/share/lqr/config"
```

### 17.3 启动

```bash
ros2 launch nav_core nav_core.launch.py
```

### 17.4 检查运行时参数

```bash
ros2 param get /nav_core target_linear_velocity
ros2 param get /nav_core wheel_base
ros2 param get /nav_core lqr_q_lateral
ros2 param get /nav_core path_acquisition_max_speed
ros2 param get /nav_core terminal_stop_distance
ros2 param get /nav_core terminal_monotonic_release_distance
ros2 param get /nav_core control_period
```

确认参数文件来源：

```bash
ros2 pkg prefix lqr
cat "$(ros2 pkg prefix lqr)/share/lqr/config/controller.yaml"
```

## 18. 使用边界

1. 模型没有显式描述轮胎侧偏角、载荷转移、地面摩擦变化和高速动力学；
2. 当前 DARE 输入矩阵采用源码中的工程等效转向灵敏度，不能直接视为精确非线性模型 Jacobian；
3. 局部路径必须具有正确点序、足够采样密度和连续终端切线；
4. 控制周期必须与实际回调周期一致；
5. 当前终点位置阈值不包含独立航向停止条件；
6. `base_link` 目标语义、定位传感器外参和私有控制路径必须全链路一致；
7. 不同底盘、轮胎、载荷、地面和固件版本必须重新辨识底盘参数；
8. 经验补偿只能修正可重复误差，不能掩盖 TF 跳变、路径方向错误或定位漂移；
9. 所有实车调参必须在限速、急停和足够安全空间下进行；
10. 本项目用于控制算法工程实现与研究验证，不替代车辆制造商安全控制和认证。

## License

本项目采用 [MIT License](./LICENSE)。
