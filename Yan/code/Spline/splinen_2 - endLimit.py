#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 21:30:04 2019

@author: self-driver
"""
import numpy as np
import matplotlib.pyplot as plt
from pylab import mpl
import math
'''
calculate A matrix
'''
def CalculateAMatrix(x,order):
    print('****************** CalculateAMatrix ****************************')
    n = order + 1
    paremeter = []
    SizeOfX = len(x) - 1
    # 根据约束1：2*（len(x) - 1 - 1）
    i = 1
    while  i <SizeOfX:
        data = np.zeros((n * (SizeOfX)))
        data1 = np.zeros((n * (SizeOfX)))
        for j in range(n):
            data[n*(i-1)+j] = x[i]**j   
            data1[n*(i)+j] = x[i]**j  
        paremeter.append(data)
        paremeter.append(data1)
        i = i+1
    print(' after  1  ,paremeter is :',len(paremeter))
    #print(' paremeter.shape ', np.array(paremeter).shape)
    # 根据约束2：2 个
    data = np.zeros((n * (SizeOfX)))
    data1 = np.zeros((n * (SizeOfX)))
    for i in range(n):
        data[i] = x[0]**i
        data1[i-n] = x[-1]**i
           
    paremeter.append(data)
    paremeter.append(data1)
    print(' after  2  ,paremeter is :',len(paremeter)) 
    # 根据约束3：  
    i = 1
    para_a = []
    para_x = []
    while  i < SizeOfX:
        j = 1
        while j < order:
        #构建x矩阵及系数矩阵
            k = j
            data_ = np.zeros((n * (SizeOfX)))
            data_x = np.zeros((n * (SizeOfX)))
            data_a = np.zeros((n * (SizeOfX)))
            while k <= (order):               
                data_[k+i*n] = -x[i]**(k - j ) * math.factorial(k)/math.factorial(k-j)
                data_[k+(i-1)*n] = x[i]**(k - j ) * math.factorial(k)/math.factorial(k-j)
                
                data_x[k+i*n] = -x[i]**(k - j ) * math.factorial(k)/math.factorial(k-j)
                data_a[k+(i-1)*n] = x[i]**(k - j ) * math.factorial(k)/math.factorial(k-j)
                
                np.set_printoptions(formatter={'float': '{: 0.0f}'.format}) 
               
                k += 1
            para_x.append(data_x)
            para_a.append(data_a)
            paremeter.append(data_)
            #print('data_ is : ',data_)
            
            j += 1
        i = i+1
    print(' after  3  ,paremeter is :',len(paremeter))
    # 根据约束4：order－１个 
    print(' order is  ',order)
    for i in range((order-1)/2):
        data = np.zeros((n * (SizeOfX)))    
        data[order-i-1]=1;
        print (i)
        print(data)
        paremeter.append(data) 
    for i in range(order-1):
        data = np.zeros((n * (SizeOfX)))    
        data[n * (SizeOfX) - i -1 ]=1;
        paremeter.append(data) 
    print(' after  4  ,paremeter is :',len(paremeter))
    return paremeter
'''
calculate b matrix
'''
def CalculatebMatrix(y,order):
    print('****************** CalculatebMatrix ****************************')
    paremeter = []
    SizeOfy = len(y) - 1
    # 共有（ｎ＋１）个点
    # 根据约束1： 2（n-１） 个公式
    i = 1
    while  i <SizeOfy:         
        paremeter.append(y[i])
        paremeter.append(y[i])
        i = i+1
    print(' after  1  ,paremeter is :',len(paremeter))
    # 根据约束2： 2 个公式
    paremeter.append(y[0])
    paremeter.append(y[-1])
    print(' after  2  ,paremeter is :',len(paremeter))
    
    # 根据约束3：  （order - 1）* (n - 1)个公式
    i = 1
    while  i < SizeOfy:
        j = 1
        while j < order:
        #构建x矩阵及系数矩阵
            paremeter.append(0)
            j += 1
        i = i+1
    print(' after  3  ,paremeter is :',len(paremeter))
    # 根据约束4： （order - 1 ）个公式 
    for i in range(order-1):        
        paremeter.append(0)
    for i in range(order-1):        
        paremeter.append(0)
        
    print(' after  4  ,paremeter is :',len(paremeter))
    return paremeter  
'''
solve linear quations
Ax = b
A : coefficient matrix
x : polynomial coefficient
b : target number
'''

def Spline2(x_,y_,order):
    #A =[list(item) for item in CalculateAMatrix(x_,order)]
    A = np.array(CalculateAMatrix(x_,order))
    b = np.array(CalculatebMatrix(y_,order))
    np.set_printoptions(formatter={'float': '{: 0.0f}'.format}) 
    print('A - shape :', A.shape) 
    #print(A)
    #for i in range(len(A)):
    #    print(A[i]) 
    print('b - shape :', b.shape)
    print('b - is    :', b)
    return np.linalg.solve(A,b)
'''
plot data
'''
def Plot(x,y,CM,order):
    n = order + 1
    
    '''
    计算插值之后的点
    '''
    new_data_x=[];
    new_data_y=[];
    for i in range(len(x)-1):
        data_x = np.arange(x[i],x[i+1], 0.1)
        data_y = np.zeros(len(data_x))
        for j in range(n):
            data_y += CM[j + i*n]  * data_x ** j
        new_data_x.extend(data_x)
        new_data_y.extend(data_y)
    data_x = np.arange(x[-1],x[-1]+0.1, 0.1)
    data_y = np.zeros(len(data_x))
    for j in range(n):
        data_y += CM[j + i*n]  * data_x ** j
    new_data_x.extend(data_x)
    new_data_y.extend(data_y)
    
    plt.plot(new_data_x, new_data_y, label="fit line", color="red")
    plt.scatter(x,y, label="point",color="red")
    mpl.rcParams['font.sans-serif'] = ['SimHei']
    mpl.rcParams['axes.unicode_minus'] = False
    plt.title("second order spline ")
    plt.legend(loc="upper left")
    plt.show()
print("**********************************************************")     
print("*************               2d                    ********")     
print("**********************************************************")     
#x = [3, 4, 7, 9,10]
#y = [2, 1, 2, 1,3]
x = [3, 4.5, 7, 9, 12, 15]
y = [2.5, 1, 2.5, 0.5, 0.5, 0.5]
n = 5
re = Spline2(x,y,n)
print('re is :',re)
#for i in range(len(x)-1):
#    print(re[i*n:(i+1)*n])
Plot(x,y,re,n)