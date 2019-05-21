#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 21:30:04 2019

@author: self-driver
"""
'''
Interpolation using quadratic spline

'''

import numpy as np
import matplotlib.pyplot as plt
from pylab import mpl
'''
calculate A matrix
'''

def CalculateAMatrix(x):
    paremeter = []
    SizeOfX = len(x) - 1
    # 根据约束1：
    i = 1
    while  i <SizeOfX:
        data = np.zeros((3 * (SizeOfX)))
        data1 = np.zeros((3 * (SizeOfX)))
        for j in range(3):
            data[3*(i-1)+j] = x[i]**(2 -j )  
            data1[3*( i)+j] = x[i]**(2-j )   
        paremeter.append(data)
        paremeter.append(data1)
        i = i+1
    # 根据约束2：
    data = np.zeros((3 * (SizeOfX)))
    data1 = np.zeros((3 * (SizeOfX)))
    
    
    for i in range(3):
        data[i] = x[0]**(2-i )  
        data1[i-3] = x[-1]**(2-i )  
            
    paremeter.append(data)
    paremeter.append(data1)
            
    # 根据约束3：  
    i = 1
    while  i <SizeOfX:
        data = np.zeros((3 * (SizeOfX)))
        for j in range(2):
            data[3*i+j] = -x[i]**(2-j-1) *(2-j)    
            data[3*(i-1)+j] = x[i]**(2-j-1) *(2-j)
        paremeter.append(data)
        i = i+1
    # 根据约束4：   
    data = np.zeros((3 * (SizeOfX)))
    data[0]=1;
    paremeter.append(data) 
    
    return paremeter
'''
calculate b matrix
'''
def CalculatebMatrix(y):
    paremeter = []
    SizeOfy = len(y) - 1
    # 根据约束1：
    i = 1
    while  i <SizeOfy:         
        paremeter.append(y[i])
        paremeter.append(y[i])
        i = i+1
    # 根据约束2：
    paremeter.append(y[0])
    paremeter.append(y[-1])
    # 根据约束3：  
    i = 1
    while  i <SizeOfy:
        paremeter.append(0)
        i = i+1
    # 根据约束4：   
    paremeter.append(0)
    #for i in range(len(paremeter)):
    #    print(paremeter[i]) 
    return paremeter   
'''
solve linear quations
Ax = b
A : coefficient matrix
x : polynomial coefficient
b : target number
'''

def Spline2(x_,y_):
    A = np.array(CalculateAMatrix(x_))
    b = np.array(CalculatebMatrix(y_))
    np.set_printoptions(formatter={'float': '{: 0.1f}'.format}) 
    print('A :') 
    print(A)
    #for i in range(len(A)):
    #    print(A[i]) 
    print('b :',b) 
    return np.linalg.solve(A,b)

print("*****************************2d*****************************")     
x = [3, 4.5, 7, 9]
y = [2.5, 1, 2.5, 0.5]
re = Spline2(x,y)
print('re is :',re)
            