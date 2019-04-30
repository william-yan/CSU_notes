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
    '''
    while(count):
        points_insert = points[-1]+(points[-1]-points[-2])*0.1*count
        points = np.insert(points,len(points),values=points_insert, axis=0)
        count = count - 1
    '''
    result = np.transpose(points);
    result_D = np.transpose(points);
    D = [0]
    for i in range(0,len(points)-1,n):
        end_selesct = i+n+1
        Path_selct = points[i:end_selesct]
        #print("Path_selct before is : \n",Path_selct)
        '''
        if (i+n+2) < (len(points)):
            point_insert_front = (points[i] + points[i+1])/2
            point_insert_rear = (Path_selct[-1] + Path_selct[-2])/2
            
            x = point_insert_rear[0]
            x1 = points[end_selesct-1][0]
            x2 = points[end_selesct][0]
            y1 = points[end_selesct-1][1]
            y2 = points[end_selesct][1]  
            
            point_insert_rear[1] = (y2-y1)/(x2-x1)*(x-x1) + y1
            Path_selct = np.insert(Path_selct,1,values=point_insert_front, axis=0)
            Path_selct = np.insert(Path_selct,-1,values=point_insert_rear, axis=0)

        else:
            point_insert_front = (points[i] + points[i+1])/2
            point_insert_rear = (Path_selct[-1] + Path_selct[-2])/2
            Path_selct = np.insert(Path_selct,1,values=point_insert_front, axis=0)
            Path_selct = np.insert(Path_selct,-1,values=point_insert_rear, axis=0)
        '''     
        
        tempresult = Bezier.bezier_curve(Path_selct,nTimes)
        tempresult_D1 = np.array([Bezier.bezier_curve(Path_selct[:-2],nTimes)])
        tempresult_D2 = np.array([Bezier.bezier_curve(Path_selct[1:],nTimes)])
        
        tempresult_D = tempresult_D2 - tempresult_D1
        print("tempresult_D.shape",tempresult_D.shape)
        for i in range (100):
            D.append(tempresult_D[0][1][i]/tempresult_D[0][1][i])

        result = np.c_[result,tempresult]
        result_D = np.append(result_D,tempresult_D)
        print("Path_selct\n",Path_selct)
        #print("Path_selct_D\n",Path_select_D)

    x_re = result[0][len(points):-1]
    y_re = result[1][len(points):-1]
    print("D is : \n",result_D)
    plt.figure(2)
    #D = np.array(bezierPoint[2])    
    plt.plot(D[1:],'k.')    
    print("x_re.shape",x_re.shape)
    #print("D.shape",D.shape)
    return x_re,y_re,D[1:]
        

if __name__ == "__main__":
    a0 = 1
    a1 = 0.5
    a2 = -0.01
    a3 = 0.000001
    x = np.linspace(0,20,10)
    y = a0 + a1 * x + a2 * x**2 + a3 * x ** 3
    Path = np.array([x,y])
    
    Path = np.transpose(Path)   

    n = 10
    bezierPoint = bezier_n_curve(Path, n, 100)
    
    plt.figure(1)
    plt.plot(x,y,'ro')
    plt.axis("equal")
    plt.plot(bezierPoint[0],bezierPoint[1],'r.')
    Path_D= np.array([bezierPoint[0]])

    
    Path_D = a1 + 2 * a2 * bezierPoint[0] + 3 * a3 *bezierPoint[0]**2
    plt.figure(2)
    #D = np.array(bezierPoint[2])
    #print(bezierPoint[2])
    #plt.plot(bezierPoint[0], D,'k.')
    plt.plot(bezierPoint[0],Path_D,'r.')

    
    
    
    



    

  
