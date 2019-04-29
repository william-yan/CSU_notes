------

# Apollo 控制代码中误差计算

Apollo中纵向控制是在Frenet坐标系下计算的，该坐标主要有两个方向：前进 方向跟垂直方向，

![5](https://img-blog.csdn.net/20180128155358389?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvZGF2aWRob3BwZXI=/font/5a6L5L2T/fontsize/100/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)



![6](https://img-blog.csdn.net/20180129145948315?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvZGF2aWRob3BwZXI=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/50/gravity/SouthEast)

![1556500954016](/home/self-driver/.config/Typora/typora-user-images/1556500954016.png)





下面式计算误差的代码：

```c++
  double dx = x - ref_point.x();
  double dy = y - ref_point.y();

  double cos_ref_theta = std::cos(ref_point.theta());
  double sin_ref_theta = std::sin(ref_point.theta());

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and  (dx, dy)
  // 计算（dx，dy）在 T 方向的投影）
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and (dx, dy)
  // 计算（dx，dy）在 N 方向的投影）
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s() + dot_rd_nd;
```





