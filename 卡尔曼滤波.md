# 使用卡尔曼滤波解算六轴数据



> 教程链接： https://www.bilibili.com/video/BV1nh411w7b4/
> 
> 核心代码在`kf_eigen.hpp`文件里



## 卡尔曼滤波

卡尔曼滤波是一种常用的状态估计方法，其基本原理是**对系统状态进行最优估计**。其主要思想是**将系统的状态分为先验状态和后验状态，通过将先验状态和观测值进行加权平均**，得到最优估计状态。

卡尔曼滤波增益的计算是**使后验误差协方差矩阵对角线上元素和（迹）最小**（协方差最小）

卡尔曼滤波**假设系统状态和测量噪声都服从零均值的高斯分布，也就是正态分布的期望值为0**。这是因为卡尔曼滤波是一种线性高斯滤波器，它假设噪声是零均值的高斯白噪声，并且系统状态的转移和测量模型都是线性的。在实际应用中，**通常可以通过对干扰的均值进行估计，然后进行补偿处理，从而使得干扰的期望值为零**。



## 系统建模：

由现代控制理论知识可知：
$$
\left\{ 
\begin{aligned}
X_{k}&=AX_{k-1}+Bu_{k-1}+W_{k} \\
Z_{k}&=HX_{k}+V_{k}
\end{aligned} 
\right.
$$
其中$A$是状态转移矩阵，$B$是输入矩阵，$H$是观测矩阵。

相比现控方程多考虑到噪声干扰。分为以下两个：

- $W_{k}$ 过程噪声：符合正态分布，期望为0，协方差矩阵为${Q}$
- $V_{k}$ 测量噪声：符合正态分布，期望为0，协方差矩阵为$R$

***

将模型考虑到卡尔曼滤波方程中：
$$
\left\{ 
\begin{aligned}
\hat{X}_{k}^{-}&=A\hat{X}_{k-1}+Bu_{k} \\
Z_{k}&=H\hat{X}_{k}
\end{aligned} 
\right.
$$
$\hat{X}_{k}^{-}$为先验估计：根据上一时刻的值估计当前时刻的值

注意到几个点：

- 对系统进行建模出现的噪声$W_{k}$和$V_{k}$不会直接出现，而是成为$Q_{k}$和$R_{k}$分别代表系统噪声协方差矩阵和观测噪声协方差矩阵
- 这里的$Z_{k}$成为了观测值（也就是实际计算中的一个输入）

后面的五大公式都是基于这个方程建立的



## 五大公式：



**预测方程两个：**
$$
\begin{aligned}
\hat{x}_{k}^{-}&=A\hat{x}_{k-1}+Bu_{k} \\
P_{k}^{-}&=AP_{k-1}A^{T}+Q
\end{aligned}
$$
预测方程和系统的物理模型有关，根据物理模型推算下一时刻的状态和协方差

其中，$\hat{x}_{k}^{-}$是先验状态估计，$A$是状态转移矩阵，$\hat{x}_{k-1}$是后验状态估计，$B$是控制输入矩阵，$u_{k}$是控制输入向量，$P_{k}^{-}$是先验误差协方差矩阵，$Q$是系统噪声协方差矩阵。

**更新方程三个：**

1. 卡尔曼增益计算方程：

$$
K_{k}=P_{k}^{-}H^{T}(HP_{k}^{-}H^{T}+R)^{-1}
$$

​        $K_{k}$是卡尔曼增益矩阵，$H$是观测矩阵，$R$是观测噪声协方差矩阵

2. 先验误差协方差矩阵更新方程：

$$
P_{k}=(I-K_{k}H)P_{k}^{-}(I-K_{k}H)^{T}+K_{k}RK_{k}^{T}
$$

或者简化形式是（两者等价）：
$$
P_{k} = (I - K_k H) P_{k}^{-}
$$

3. 状态量更新方程：

$$
\hat{x}_{k} = \hat{x}_{k}^{-} + K_k(z_k - H \hat{x}_{k}^{-})
$$






## 实例：六轴加速度陀螺仪姿态解算

融合三轴加速度计和三轴角速度计

- 对三轴角速度计进行建模，构建两个预测方程
- 使用三轴加速度计进行观测

***

#### 首先对系统进行建模：


$$
\begin{aligned}
&
\begin{bmatrix} roll\_gyro_{k}^{-}  \\ pitch\_gyro_{k}^{-} \\ yaw\_gyro_{k}^{-}\end{bmatrix} = 
\begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0\\ 0 & 0 & 1 \end{bmatrix} 
\begin{bmatrix} roll\_gyro_{k-1}  \\ pitch\_gyro_{k-1}\\ yaw\_gyro_{k-1}\end{bmatrix}+ 
\begin{bmatrix} {\Delta}t & 0 & 0 \\  0 & {\Delta}t & 0 \\ 0 & 0 & {\Delta}t \end{bmatrix}
\begin{bmatrix} gyro\_x \\ gyro\_y \\ gyro\_z \end{bmatrix} \\
&
\begin{bmatrix} roll\_acc  \\ pitch\_acc \\ yaw\_acc\end{bmatrix}
=\begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0\\ 0 & 0 & 0\end{bmatrix}
\begin{bmatrix} roll\_esti_{k}  \\ pitch\_esti_{k} \\ yaw\_esti_{k}\end{bmatrix}

\end{aligned}
$$


状态量为$\begin{bmatrix} roll\_gyro  \\ pitch\_gyro\\ yaw\_gyro\end{bmatrix}$，含义直接为角度计算出来的姿态角（只和角速度有关）

状态转移矩阵为$\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0& 0& 1\end{bmatrix}$

输入直接是角速度

$Z_{k}$为为加速度计求得的姿态角

$\begin{bmatrix} roll\_esti_{k}  \\ pitch\_esti_{k} \\ yaw\_esti_{k}\end{bmatrix}$为由状态量和观测量估计的值

***

#### 五个方程计算：



$\hat{x}_{k}^{-}=A\hat{x}_{k-1}+Bu_{k}$对应的方程为：
$$
\begin{bmatrix} roll\_gyro_{k}  \\ pitch\_gyro_{k}\\ yaw\_gyro_{k}\end{bmatrix} = 
\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0& 1\end{bmatrix} 
\begin{bmatrix} roll\_gyro_{k-1}  \\ pitch\_gyro_{k-1}\\ yaw\_gyro_{k-1}\end{bmatrix}+ 
\begin{bmatrix} {\Delta}t & 0 & 0  \\  0 & {\Delta}t & 0\\  0 & 0& {\Delta}t\end{bmatrix}
\begin{bmatrix} gyro\_x \\ gyro\_y\\ gyro\_z \end{bmatrix}
$$

根据建模内容，可以知道：

$A$状态转移矩阵为$\begin{bmatrix} 1 & 0 & 0 \\ 0 &  1& 0 \\ 0 & 0& 1\end{bmatrix} $，$B$=$\begin{bmatrix} {\Delta}t & 0 & 0 \\  0 & {\Delta}t & 0\\  0 & 0&  {\Delta}t\end{bmatrix}$，其中${\Delta}t$为两次循环时间计算间隔

$\begin{bmatrix} gyro\_x \\ gyro\_y\\ gyro\_z \end{bmatrix}$为输入对应轴上的角速度，$\begin{bmatrix} roll\_gyro_{k} \\pitch\_gyro_{k}\\yaw\_gyro_{k}\end{bmatrix}$为由角速度计算出的状态量

***

$P_{k}^{-}=AP_{k-1}A^{T}+Q$对应的方程为：
$$
P_{k}^{-}=
\begin{bmatrix} 1 & 0 & 0 \\ 0 & 1&  0 \\ 0 & 0& 1\end{bmatrix}
P_{k-1}
\begin{bmatrix} 1 & 0 & 0 \\ 0 & 1&  0 \\ 0 & 0& 1\end{bmatrix}
+ 
Q
$$

***

$K_{k}=P_{k}^{-}H^{T}(HP_{k}^{-}H^{T}+R)^{-1}$对应的方程为：
$$
\begin{aligned}
K_{k}&=P_{k}^{-}\begin{bmatrix}  1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0& 0\end{bmatrix}
(
\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0& 0\end{bmatrix}
P_{k}^{-}
\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0& 0\end{bmatrix}+
R
)^{-1} \\
&=P_{k}^{-}(P_{k}^{-}+R)^{-1}
\end{aligned}
$$

***

$P_{k} = (I - K_k H) P_{k}^{-}$对应的方程为：

$$
\begin{aligned}
P_{k} &= ( 
\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0& 1\end{bmatrix}
- K_k 
\begin{bmatrix}  1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0& 0\end{bmatrix}
) P_{k}^{-}
\end{aligned}
$$

***

$\hat{x}_{k} = \hat{x}_{k}^{-} + K_k(z_k - H \hat{x}_{k}^{-})$对应的方程为：
$$
\hat{x}_{k} = \hat{x}_{k}^{-} + K_k(
\begin{bmatrix} roll\_acc  \\ pitch\_acc\\ yaw\_acc\end{bmatrix}
- 
\begin{bmatrix} 1 & 0 & 0  \\ 0 & 1 & 0 \\ 0& 0 & 0\end{bmatrix}
\hat{x}_{k}^{-})
$$

***

#### 初始化值

$Q=\begin{bmatrix} 0.002 & 0 & 0 \\ 0 & 0.002 & 0  \\ 0 & 0 & 0.002 \end{bmatrix}$

$R=\begin{bmatrix} 0.2 & 0 & 0 \\ 0 & 0.2 & 0 \\ 0 & 0 & 0.2 \end{bmatrix}$

$P_{0}=\begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 &  0 \\ 0 & 0 & 1\end{bmatrix}$

$x_{0}=\begin{bmatrix}  0 \\ 0 \\ 0 \end{bmatrix}$
