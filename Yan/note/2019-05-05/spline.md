# 三次样条插值

## 1. 原理

假设有

$x: x_0<x_1<...<x_{n-1}<x_n$,

$y: y_0,y_1,...,y_{n-1},y_n$

### 1.1 定义

样条曲线是一个分段定义的公式，给定n+1个数据，共有n个区间，满足：

（1）在每一个分段区间$[x_i,x_{i+1}]$中,$S(x)=S_i(x)$都是一个三次多项式

（2）端点值相等，即$S(x_i)=y_i$

（3）$S(x)$，导数$S'(x)$、二阶导$S''(x)$在【a,b】内连续，即$S(x)$曲线是光滑的

n个三次多项式分段可以写作：
$$
S_i(x)=a_i+b_i(x-x_i)+c_i(x-x_i)^2+d_i(x-x_i)^3
$$
其中，$a_i,b_i,c_i,d_i$为未知系数

### 1.2求解

~~~
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 16:27:41 2019

@author: self-driver
"""

import numpy as np
import matplotlib.pyplot as plt
from pylab import mpl

'''
二次样条实现：
函数x的自变量为:3,   4.5, 7,    9
      因变量为：2.5, 1   2.5,  0.5
'''

x = [3, 4.5, 7, 9]
y = [2.5, 1, 2.5, 0.5]

"""
功能：完后对二次样条函数求解方程参数的输入
参数：要进行二次样条曲线计算的自变量
返回值：方程的参数
"""

def calculateEquationParameters(x):
    #parameter为二维数组，用来存放参数，sizeOfInterval是用来存放区间的个数
    parameter = []
    sizeOfInterval=len(x)-1;
    i = 1
    #首先输入方程两边相邻节点处函数值相等的方程为2n-2个方程
    while i < len(x)-1:
        data = init(sizeOfInterval*3)
        data[(i-1)*3]=x[i]*x[i]
        data[(i-1)*3+1]=x[i]
        data[(i-1)*3+2]=1
        data1 =init(sizeOfInterval*3)
        data1[i * 3] = x[i] * x[i]
        data1[i * 3 + 1] = x[i]
        data1[i * 3 + 2] = 1
        temp=data[1:]
        parameter.append(temp)
        temp=data1[1:]
        parameter.append(temp)
        i += 1
    #输入端点处的函数值。为两个方程,加上前面的2n-2个方程，一共2n个方程
    data = init(sizeOfInterval*3-1)
    data[0] = x[0]
    data[1] = 1
    parameter.append(data)
    data = init(sizeOfInterval *3)
    data[(sizeOfInterval-1)*3+0] = x[-1] * x[-1]
    data[(sizeOfInterval-1)*3+1] = x[-1]
    data[(sizeOfInterval-1)*3+2] = 1
    temp=data[1:]
    parameter.append(temp)
    #端点函数值相等为n-1个方程。加上前面的方程为3n-1个方程,最后一个方程为a1=0总共为3n个方程
    i=1
    while i < len(x) - 1:
        data = init(sizeOfInterval * 3)
        data[(i - 1) * 3] =2*x[i]
        data[(i - 1) * 3 + 1] =1
        data[i*3]=-2*x[i]
        data[i*3+1]=-1
        temp=data[1:]
        parameter.append(temp)
        i += 1
    return parameter



"""
对一个size大小的元组初始化为0
"""
def init(size):
    j = 0
    data = []
    while j<size:
        data.append(0)
        j += 1
    return data


"""
功能：计算样条函数的系数。
参数：parametes为方程的系数，y为要插值函数的因变量。
返回值：二次插值函数的系数。
"""
def solutionOfEquation(x_,y_):
    sizeOfInterval = len(x_) - 1;
    result = init(sizeOfInterval*3-1)
    i=1
    while i<sizeOfInterval:
        result[(i-1)*2]=y_[i]
        result[(i-1)*2+1]=y_[i]
        i+=1
    result[(sizeOfInterval-1)*2]=y_[0]
    result[(sizeOfInterval-1)*2+1]=y_[-1]
    a = np.array(calculateEquationParameters(x))
    b = np.array(result)
    np.set_printoptions(formatter={'float': '{: 0.1f}'.format}) 
    print("a is ")
    print(a)
    print("b is ")
    np.set_printoptions(formatter={'float': '{: 0.1f}'.format}) 
    print(b)
    
    return np.linalg.solve(a,b)

"""
功能：根据所给参数，计算二次函数的函数值：
参数:parameters为二次函数的系数，x为自变量
返回值：为函数的因变量
"""
def calculate(paremeters,x):
    result=[]
    for data_x in x:
        result.append(paremeters[0]*data_x*data_x+paremeters[1]*data_x+paremeters[2])
    return result


"""
功能：将函数绘制成图像
参数：data_x,data_y为离散的点.new_data_x,new_data_y为由拉格朗日插值函数计算的值。x为函数的预测值。
返回值：空
"""
def  Draw(data_x,data_y,new_data_x,new_data_y):
        plt.plot(new_data_x, new_data_y, label="fit line", color="black")
        plt.scatter(data_x,data_y, label="point",color="red")
        mpl.rcParams['font.sans-serif'] = ['SimHei']
        mpl.rcParams['axes.unicode_minus'] = False
        plt.title("second order spline ")
        plt.legend(loc="upper left")
        plt.show()

result=solutionOfEquation(x,y)
new_data_x1=np.arange(3, 4.5, 0.1)
new_data_y1=calculate([0,result[0],result[1]],new_data_x1)
new_data_x2=np.arange(4.5, 7, 0.1)
new_data_y2=calculate([result[2],result[3],result[4]],new_data_x2)
new_data_x3=np.arange(7, 9.5, 0.1)
new_data_y3=calculate([result[5],result[6],result[7]],new_data_x3)
new_data_x=[]
new_data_y=[]
new_data_x.extend(new_data_x1)
new_data_x.extend(new_data_x2)
new_data_x.extend(new_data_x3)
print(new_data_y1)
print(result)

new_data_y.extend(new_data_y1)
new_data_y.extend(new_data_y2)
new_data_y.extend(new_data_y3)
Draw(x,y,new_data_x,new_data_y)
~~~

