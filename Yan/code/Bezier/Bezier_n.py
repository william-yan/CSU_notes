#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 25 16:40:54 2019

@author: self-driver
"""
import numpy as np
from matplotlib import pyplot as plt
import Bezier

def bezier_n_curve(points, n, nTimes=1000):
    
    #print("points is : \n",points)
    #print("   n   is : ",n)
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
    print("points is : \n",points)
    nPoints = len(points)
    count = 0 
    if((nPoints - 1)%n):        
        count = n - (nPoints - 1)%n
    print('count is : ',count)
    # insert Point
    while(count):
        points_insert = points[-1]
        points = np.insert(points,len(points),values=points_insert, axis=0)
        count = count - 1
    
    result = np.transpose(points);
    result_D = np.transpose(points);
    for i in range(0,len(points)-1,n):
        end_selesct = i+n+1
        Path_selct = points[i:end_selesct]
        print("Path_selct before is : \n",Path_selct)

        if (i+n+2) < (len(points)):
            point_insert_front = (points[i] + points[i+1])/2
            point_insert_rear = (Path_selct[-1] + Path_selct[-2])/2
            
            x = point_insert_rear[0]
            x1 = points[end_selesct-1][0]
            x2 = points[end_selesct][0]
            y1 = points[end_selesct-1][1]
            y2 = points[end_selesct][1]  
            
            point_insert_rear[1] = (y2-y1)/(x2-x1)*(x-x1) + y1
            '''
            print("x is :",x)
            print("x1 is :",x1)
            print("x2 is :",x2)
            print("y is :",point_insert_rear[1])
            print("y1 is :",y1)
            print("y2 is :",y2)
            plt.figure(1)
            plt.plot(x,point_insert_rear[1],'ko')
            plt.plot(x1,y1,'go')
            plt.plot(x2,y2,'bo')
            '''
            Path_selct = np.insert(Path_selct,1,values=point_insert_front, axis=0)
            Path_selct = np.insert(Path_selct,-1,values=point_insert_rear, axis=0)

        else:
            point_insert_front = (points[i] + points[i+1])/2
            point_insert_rear = (Path_selct[-1] + Path_selct[-2])/2
            Path_selct = np.insert(Path_selct,1,values=point_insert_front, axis=0)
            Path_selct = np.insert(Path_selct,-1,values=point_insert_rear, axis=0)
        print("point_insert_front  is : \n",point_insert_front)
        print("point_insert_rear   is : \n",point_insert_rear)
        print("Path_selct after is : \n",Path_selct)
        tempresult = Bezier.bezier_curve(Path_selct,nTimes)
        result = np.c_[result,tempresult]
        # Derivative n-1 order Bezier curve
        Path_select_D = Path_selct[1:] - Path_selct[:-1]
        tempresult_D = Bezier.bezier_curve(Path_select_D,nTimes)
        result_D = np.c_[result_D,tempresult_D]   
        
        #plt.figure(1)
        #plt.plot(Path_selct[-2][0],Path_selct[-2][1],'ko')
        #plt.plot(Path_selct[-1][0],Path_selct[-1][1],'go')
        

    #plt.figure(1)
    #plt.plot(Path_selct[0],Path_selct[1],'b.')
    #plt.plot(result[0][len(points):-1],result[1][len(points):-1],'b.')
    #print('result[0][len(points):-1]:\n',result[0][len(points):-1])
    #print('result[1][len(points):-1]:\n',result[1][len(points):-1])

    x_re = result[0][len(points):-1]
    y_re = result[1][len(points):-1]
    x_re_D = result_D[0][len(points):-1]
    y_re_D = result_D[1][len(points):-1]
    print(result_D[0][len(points):-1])
    print(result_D[1][len(points):-1])
                 
    return x_re,y_re,x_re_D,y_re_D
        
    # insert point
    
if __name__ == "__main__":
    a0 = 1
    a1 = 0.5
    a2 = -0.01
    a3 = 0.000001
    x = np.linspace(0,20,10)
    y = a0 + a1 * x + a2 * x**2 + a3 * x ** 3
    Path = np.array([x,y])
    
    Path = np.transpose(Path)   

    n = 5
    bezierPoint = bezier_n_curve(Path, n, 100)
    
    plt.figure(1)
    plt.plot(x,y,'ro')
    plt.axis("equal")
    plt.plot(bezierPoint[0],bezierPoint[1],'r.')
    plt.figure(2)
    plt.plot(bezierPoint[0],bezierPoint[3],'r.')
    
    
    



    

  
