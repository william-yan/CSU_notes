#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 10 21:16:47 2019

@author: self-driver
"""
import numpy as np
import matplotlib.pyplot as plt
from pylab import mpl
import math

'''
x = [3, 4.5, 7, 9]
SizeOfX = len(x)
paremeter = []
i = 1
while  i < SizeOfX:
    
    data = np.zeros((n * (SizeOfX)))
    for j in range(n-1):
        data[n*i+j+1] = -x[i]**j*(j+1)     
        data[n*(i-1)+j+1] = x[i]**j*(j+1) 
    paremeter.append(data)
    i = i+1
'''

order = 4
para_a = []
para_x = []
para = []
for i in range(order):
    #构建x矩阵及系数矩阵
    j = i
    data_x = np.zeros(order+1)
    data_a = np.zeros(order+1)
    data = np.zeros(order+1)
    while j <= (order):   
        data_x[j] = j - i   
        data_a[j] = math.factorial(j)/math.factorial(j-i)
        data [j] = 2**(j-i) * math.factorial(j)/math.factorial(j-i)
        j += 1
    para_x.append(data_x)
    para_a.append(data_a)
    para.append(data)
    #构建系数矩阵
    
    
print('para_a :') 
for i in range(len(para_a)):
    print(para_a[i]) 
print('para_x :') 
for i in range(len(para_x)):
    print(para_x[i]) 
print('para :') 
for i in range(len(para)):
    print(para[i]) 
    
