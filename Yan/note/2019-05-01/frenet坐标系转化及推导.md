[TOC]

# Frenet坐标系转化及推导

## 1. 坐标系定义

**主要是参考《Multivariate Calculus and Geometry》第七章的内容自行解读及《高等数学 第Ⅲ卷：多元微积分与微分几何初步》中相关内容进行解释，建议看英文的，中文的解释有点儿勉强，但是英文的比较难懂。**

定义$P：[x(t),y(t)]$是一个二维向量，则$P'(t)=[x'(t),y'(t)]$

定义 ： $T(t) = P'(t) = [x'(t),y'(t)]$

将向量T(t)逆时针旋转90°可得到一个与向量T(t)垂直的向量N(t)

定义： $N(t) = [-y'(t),x'(t)]$

根据T的定义可知：$<T(t),T(t)> = 1$
$$
\frac{d<T(t),T(t)>}{dt}=0=<T'(t),T(t)>+<T(t),T'(t)>        \tag{1-1}
$$
因此：
$$
<T'(t),T(t)>=<T(t),T'(t)>=0
$$
根据上式可以知道：T'(t)与T(t)垂直，则T'(t)与N(t)平行:
$$
T'(t)=\kappa(t)N(t)
$$
根据点乘的一个集合意义：一个向量在另外一个向量的投影：
$$
\kappa(t)  =  <T'(t),N(t)> =<k(t)N(t),N(t)> \\=(x''(t),y''(t))\cdot(-y'(t),x(t))\\
      =    x'(t)y''(t)-x''(t)y'(t)        \tag{1-2}
$$
$$
s(t) = \int_a^t ||P'(x)||dxdx
$$

对于上式来说，s：[a，b]->[0,l]，s‘(t)=||P'(t)||>0，s为单调递增，对s的反函数s^-1
$$
(s^{-1})'(t)=\frac{1}{s'(s^{-1}(t))}=\frac{1}{||P'(s^{-1}(t))||}        \tag{1-3}
$$
因此：
$$
||(P\circ s^{-1})'(t)||
=\frac{||P'(s^{-1}(t))||}{s'(s^{-1})'(t)}

=\frac{||P'(s^{-1}(t))||}{||P'(s^{-1}(t))||}=1          \tag{1-4}
$$
则：
$$
(x\circ s^{-1})'(t)=x'((s^{-1})'(t))\cdot(s^{-1})'(t)\\
(x\circ s^{-1})''(t)=x''[(s^{-1})'(t)]\cdot(s^{-1})'(t)^2 +
                     x'(s^{-1}(t))'\cdot(s^{-1}) ''(t)
$$

对于y，同样可以得到$(x\circ s^{-1})''(t)$跟$(x\circ s^{-1})'(t)$.
$$
(y\circ s^{-1})'(t)=y'((s^{-1})'(t))\cdot(s^{-1})'(t)\\
(y\circ s^{-1})''(t)=y''[(s^{-1})'(t)]\cdot(s^{-1})'(t)^2 +
                     y'(s^{-1}(t))'\cdot(s^{-1}) ''(t)
$$
将公式（8）跟（9）带入公式（4）并化简得到：
$$
\kappa(t)=\frac{x'(t)y''(t)-x''(t)y'(t)}{((x'(t))^2+(y'(t))^2)^{\frac{3}{2}}}
$$
对于
$$
B'(t)=-\tau(t)N(t)        \tag{1-5}
$$
其中有一个 - 是为了避免后面引入更多负号。$\tau$是扰率
$$
N(t)=B(t)\times T(t)\\
N'(t)=B'(t)\times T(t)+B(t)\times T'(t)
=-\tau(t)N(t)\times T(t)+B(t)\times k(t)N(t)
\\=-\tau(t)B(t)+ \kappa(t)T(t)        \tag{1-6}
$$
至此，我们就有了Frenet坐标系的定义：
$$
T'(t)=\kappa(t)N(t)\\
B'(t)=-\tau(t)N(t)\\
N'(t)=-\tau(t)B(t)+ \kappa(t)T(t)        \tag{1-7}
$$

$$
\begin{bmatrix}   T'(t)\\    N'(t)\\    B'(t)\\ \end{bmatrix} = 
\begin{bmatrix}   0& \kappa & 0\\   -\kappa & 0 &\tau \\ 0 &-\tau&0   \\ \end{bmatrix}\begin{bmatrix}   T(t)\\    N(t)\\    B(t)\\ \end{bmatrix}
$$



## 2.笛卡尔坐标系与转化为Frenet 坐标系

**参考：https://blog.csdn.net/davidhopper/article/details/79162385进行推导**

如图所示，图中$\vec{x}=(x,y)$为车辆当前位置，$\vec{r}$为参考线上一点，这里采用弧长$s$与横向偏移$l$表示。

![20180129145948315](/home/self-driver/图片/20180129145948315.png)

令$\theta_{\vec{r}},\vec{T_{\vec{r}}},\vec{N_{\vec{r}}}$为当前参考线$\vec{r}(s)$的方位角、单位切向量和法向量，$\theta_{\vec{x}},\vec{T_{\vec{x}}},\vec{N_{\vec{x}}}$为当前轨迹点$\vec{x}(s,l)$的方位角，单位切向量和范围法向量，根据正焦急的定义：
$$
\vec {T_{\vec{r}}}= [\space\space\space\space cos\theta_{\vec{r}}   \space\space sin\theta_{\vec{r}}]\\
\vec {N_{\vec{r}}}= [-sin\theta_{\vec{r}}\space\space cos\theta_{\vec{r}}]\\
\vec {T_{\vec{x}}}= [\space\space\space\space cos\theta_{\vec{x}}\space\space sin\theta_{\vec{x}}]\\       
\vec {N_{\vec{x}}}= [-sin\theta_{\vec{x}}\space\space cos\theta_{\vec{x}}] \tag{2-1}
$$
**(1)求$l$**

$l(s)$为向量$(\vec{r}-\vec{x})$在向量$\vec{N_{\vec{r}}}$上的投影，即：
$$
l=[\vec{r}-\vec{x}]^T\vec{N_{\vec{r}}}        \tag{2-2}
$$
但实际应用中实际的参考位置可能选取的是最近的点，所以Apollo中采用的是如下的计算公式：
$$
l=\sqrt{(x_{\vec{x}}-x_{\vec{r}})^2+(y_{\vec{x}}-y_{\vec{r}})^2}
$$
代码如下：

```c++
//目录： math/cartesian_frenet_conversion.h：
const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
ptr_d_condition->at(0) = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
```

**备注：以下所有标记中**$\dot{a}=\frac{da}{dt},\ddot{a}=\frac{d\dot{a}}{dt},d'(t)=\frac{a}{ds},d''(t)=\frac{a'}{ds}$

**(2)求$ l'$ **
$$
\dot{l}=[\dot{\vec{r}}-\dot{\vec{x}}]^T\vec{N_{\vec{r}}}+[\vec{r}-\vec{x}]^T\dot{\vec{N_{\vec{r}}}}        \tag{2-3}
$$
其中：
$$
\dot{\vec{r}}=\dot{s}\vec{T_{\vec{r}}}     \\
\dot{\vec{x}}=v_{\vec{x}}\vec{T_{\vec{x}}}   \\
\dot{\vec{N_{\vec{r}}}}=\frac{d(\vec{N_{\vec{r}}})}{d(s)} \frac {d(s)}{dt}\\
=N'_{\vec{r}}\dot{s}
$$
则：
$$
\dot{l}=[\dot{s}\vec{T_{\vec{r}}}-v_{\vec{x}}\vec{T_{\vec{x}}}]^T\vec{N_{\vec{r}}}+[\vec{r}-\vec{x}]^TN'_{\vec{r}}\dot{s}        \tag{2-4}
$$
由于$\vec{T_{\vec{r}}}^T\vec{N_{\vec{r}}}=0,\vec{T_{\vec{x}}}^T\vec{N_{\vec{x}}}=0$，定义$\Delta\theta=\theta_\vec{x}-\theta_\vec{r}$所以：
$$
\dot{l}=v_{\vec{x}}\vec{T_{\vec{x}}}^T\vec{N_{\vec{r}}}\\
=v_{\vec{x}}[-cos\theta_{\vec{x}}sin\theta_{\vec{r}}+
sin\theta_{\vec{x}}cos\theta_{\vec{r}}]\\
=v_{\vec{x}}sin(\Delta\theta)
$$
根据定义：$l'=\frac{d(l)}{d(s)}=\frac{d(l)}{dt}\frac{d(s)}{dt}=\frac{\dot{l}}{\dot{s}}$，上面已经求得$\dot{l}$,其中$\vec{T_{\vec{x}}}，\vec{N_{\vec{r}}}$为已知量，只要求出$v_{\vec{x}}$便可，又根据定义$v_{\vec{x}}=\|\dot{\vec{x}}\|$,且$\vec{x}=\vec{r}+l(s)\vec{N_{\vec{r}}}$:
$$
\dot{\vec{x}}=\frac{d(\vec{r}+l(s)\vec{N_{\vec{r}}})}{dt}
=\dot{\vec{r}} +\dot{l}(s)\vec{N_{\vec{r}}}+l(s)\dot{\vec{N_{\vec{r}}}}\\
=\dot{s}(1-\kappa_{\vec{r}}l）\vec {T_{\vec{r}}}+\dot{l}(s)\vec{N_{\vec{r}}} \\
$$

$$
v_{\vec{x}}=\|\dot{\vec{x}}\|=\sqrt{\dot{\vec{x}}^T\dot{\vec{x}}}\\
=\sqrt{[\dot{s}(1-\kappa_{\vec{r}}l）]^2+\dot{l}^2(s)}
$$

$$
l'=\frac{v_{\vec{x}}sin(\theta_\vec{x}-\theta_\vec{r})}{\dot{s}}\\
=\frac{\sqrt{[\dot{s}(1-\kappa_{\vec{r}} l）]^2+\dot{l}^2(s)}\cdot sin(\theta_\vec{x}-\theta_\vec{r})}{\dot{s}}\\
=\sqrt{[(1-\kappa_{\vec{r}} l）]^2+(\frac{\dot{l}}{\dot{s}})^2}\cdot sin(\theta_\vec{x}-\theta_\vec{r})\\
=\sqrt{[(1-\kappa_{\vec{r}} l)]^2+l'^2}\cdot sin(\theta_\vec{x}-\theta_\vec{r})
$$

则$l'$为
$$
l'=(1-\kappa_{\vec{r}} l)\cdot tan(\theta_\vec{x}-\theta_\vec{r})        \tag{2-6}
$$
```c++
const double delta_theta = theta - rtheta; 
const double tan_delta_theta = std::tan(delta_theta); 
const double cos_delta_theta = std::cos(delta_theta); 
const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0); 
// 求解d' = dd / ds 
ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta; const double kappa_r_d_prime = rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1); 
```

**(3)求$ \dot s$**
$$
(1-\kappa_{\vec{r}}l)\cdot tan(\Delta\theta)=\frac{v_{\vec{x}}}{\dot{s}}\cdot sin(\Delta\theta)\\
$$

$$
v_{\vec{x}}=\frac{\dot{s}\cdot {(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}        \tag{2-7}
$$
有了上式之后，便可求得$\dot s$
$$
{\dot{s}=\frac{v_{\vec{x}}{cos(\Delta\theta)}}{{(1-\kappa_{\vec{r}} l)}} }        \tag{2-8}
$$

```c++
// 求解s
  ptr_s_condition->at(0) = rs;
// 求解ds / dt
  ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;
```



**（4）求$l''$**

对$l''$是直接对$l'$对s进行求导：
$$
l''=\frac{d(l')}{d(s)}
=\frac{d((1-\kappa_{\vec{r}} l)\cdot tan(\Delta\theta))}{d(s)}
=(1-\kappa_{\vec{r}} l)'\cdot tan(\Delta\theta))+
(1-\kappa_{\vec{r}}l)\cdot tan'(\Delta\theta)  \tag{2-9}
$$
先求第一项：$\frac{d(1-\kappa_{\vec{r}}l )}{ds}=-(\frac{\dot\kappa_{\vec{r}}}{\dot{s}}l+\kappa_{\vec{r}}l ')$ ,$l'$上面已知，不用再求

之后是：$ tan'(\theta_\vec{x}-\theta_\vec{r}))$
$$
tan'(\Delta\theta))=\frac{\theta'_\vec{x}-\theta'_\vec{r}}{cos^2(\Delta\theta))}
$$
其中：$\theta'_\vec{r}=\kappa_{\vec{r}}$，主要是对$\theta'_\vec{x}$求s的导数。
$$
\frac{d}{d(s)}=
\frac{d}{d(s_{\vec{x}})}
\frac{d(s_{\vec{x}})}{d(t)}
\frac{d(t)}{d(s)}
=\frac{d{}}{d(s_{\vec{x}})}\cdot v_{\vec{x}} \cdot \frac{1}{\dot{s}}
=\frac{{(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}
\frac{d{}}{d(s_{\vec{x}})}
$$

$$
\frac{d{\theta_\vec{x}}}{d(s)}=
\frac{{(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}
\frac{d{\theta_\vec{x}}}{d(s_{\vec{x}})}
=\frac{{(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}
\kappa_{\vec{x}}
$$
则：$tan'(\theta_\vec{x}-\theta_\vec{r}))=\frac{\frac{{(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}
\kappa_{\vec{x}}-\kappa_{\vec{r}}}{cos^2(\Delta\theta))}$，至此，$l''$便求得。
$$
l''=-(\frac{\dot\kappa_{\vec{r}}}{\dot{s}}l+\kappa_{\vec{r}}l ')\cdot tan(\Delta\theta)+
(1-\kappa_{\vec{r}}l)\cdot \frac{\frac{{(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}
\kappa_{\vec{x}}-\kappa_{\vec{r}}}{cos^2(\Delta\theta))}\tag{2-10}
$$


```c++
 ptr_d_condition->at(2) =  -kappa_r_d_prime * tan_delta_theta + 
 one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta * (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);
```

**(5)求$\ddot s$.**


$$
a_{\vec{x}}=\frac{d(v_{\vec{x}})}{d(t)}=
{\frac{\ddot{s}\cdot {(1-\kappa_{\vec{r}}l)}}{cos(\Delta\theta)})}+\dot{s}\frac{{d(\frac{{(1-\kappa_{\vec{r}}l)}}{cos(\Delta\theta)})}}{d(s)} \frac{d(s)}{d(t)}\\
={\frac{\ddot{s}\cdot {(1-\kappa_{\vec{r}}l)}}{cos(\Delta\theta)}}+

\frac{\dot{s}^2}{{cos}^2(\Delta\theta)}

[-(\kappa'_{\vec{r}}l+\kappa_{\vec{r}}l')+
sin(\theta_\vec{x}-\theta_\vec{r})\cdot ((1-\kappa_{\vec{r}}l)\cdot
(\theta'_\vec{x}-\theta'_\vec{r})))]\tag{2-11}
$$

有了（2-11）便可求得$\ddot s$：
$$
\ddot s=
\frac{[a_{\vec{x}}-\frac{\dot{s}^2}{{cos}^2(\Delta\theta)}
[-(\kappa'_{\vec{r}}l+\kappa_{\vec{r}}l')+sin(\theta_\vec{x}-\theta_\vec{r})\cdot (1-\kappa_{\vec{r}}l)\cdot(\theta'_\vec{x}-\theta'_\vec{r}))]]
cos(\Delta\theta)}{(1-\kappa_{\vec{r}}l)}
\tag{2-12}
$$

```c++
const double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa; 
// 求解d(ds) / dt 
ptr_s_condition->at(2) = (a * cos_delta_theta - ptr_s_condition->at(1) * ptr_s_condition->at(1) * (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d;
```

至此，$l,l',l'',v_{\vec{x}},a_{\vec{x}}$便全部求得,即将坐标系转化到frenet坐标系下。

## 3.Frenet 坐标系与转化为笛卡尔坐标系

frenet坐标系转化为迪卡尔坐标系是已知frenet坐标系下相关数据，$（r_s,r_x,r_y,r_{\theta,k_{r}},k'_{r}，[l,l',l''],[s,s',s'']）$求迪卡尔坐标系下$[x,y,\theta_{\vec{x}},v,a]$，公式在上面已经推到过，这里就不再写了。

**(1)x/y坐标**
$$
x = rx-l*sin(\theta_{\vec{r}})\\ \tag{3-1}
y = ry+l*cos(\theta_{\vec{r}})
$$
```c++
  *ptr_x = rx - sin_theta_r * d_condition[0];
  *ptr_y = ry + cos_theta_r * d_condition[0];
```

**(2）航向**
$$
\Delta\theta=\arctan(\frac{l'}{1-k_{\vec{r}}l})
$$

$$
\theta_{\vec{x}}=r_{\theta_{\vec r}}+\Delta\theta\tag{3-2}
$$

```c++
  const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

  const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
  const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
  const double cos_delta_theta = std::cos(delta_theta);

  *ptr_theta = NormalizeAngle(delta_theta + rtheta);
```

**(3)速度**

由公式（31）求$k_{\vec{x}}$
$$
\kappa_{\vec{x}}=\frac
{
(\frac{
l''+(
{\kappa'_{\vec{r}}}l+\kappa_{\vec{r}}l ')\cdot
tan(\Delta\theta))
}
{(1-\kappa_{\vec{r}}l)}

cos^2(\Delta\theta)+
\kappa_{\vec{r}})\cdot{cos(\Delta\theta)}
}
{(1-\kappa_{\vec{r}} l)}\tag{3-3}
$$
根据公式（21）下面的定义：
$$
\dot l =l'\dot s \tag{3-4}
$$
则速度：
$$
v=\sqrt{((1-k_{\vec{r}}l)\cdot \dot s)^2+\dot l^2}\tag{3-5}
$$
按照公式（2-7）应该是：
$$
v_{\vec{x}}=\frac{\dot{s}\cdot {(1-\kappa_{\vec{r}} l)}}{cos(\Delta\theta)}
$$

$$
v=\sqrt{(\frac{(1-k_{\vec{r}}l)\cdot \dot s}{cos(\Delta\theta)})^2+\dot l^2}\tag{3-5}
$$

有木有大神知道为啥Apollo里面式按照公式（3-5）计算的啊

```c++

  const double kappa_r_d_prime =  rdkappa * d_condition[0] + rkappa * d_condition[1];
  *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                 cos_delta_theta * cos_delta_theta) / (one_minus_kappa_r_d)
                + rkappa) *
               cos_delta_theta / (one_minus_kappa_r_d);

  const double d_dot = d_condition[1] * s_condition[1];
  *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                         s_condition[1] * s_condition[1] +
                     d_dot * d_dot);
```

**(4)加速度**

$$
a_{\vec{x}}={\frac{\ddot{s}\cdot {(1-\kappa_{\vec{r}}l)}}{cos(\Delta\theta)}}+

\frac{\dot{s}^2}{{cos}^2(\Delta\theta)}

[-(\kappa'_{\vec{r}}l+\kappa_{\vec{r}}l')+
sin(\Delta\theta)\cdot (1-\kappa_{\vec{r}}l)\cdot
(\Delta\theta'))]\tag{2-11}
$$

```c++
  const double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa)
      - rkappa;

  *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
           s_condition[1] * s_condition[1] / cos_delta_theta *
           (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
```

