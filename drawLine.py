#!/usr/bin/env python
# coding: utf-8

# In[2]:


import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
get_ipython().run_line_magic('matplotlib', 'inline')
from IPython.display import Image
import matplotlib.pyplot as plt


# In[105]:


def drawLine(img,x,y,slope,b,g,r,thickness):
    height = img.shape[0]
    width = img.shape[1]
    points = []
    
    intercept = y-slope*x
    y0 = intercept
    x0 = -intercept/slope
    ymax = intercept + slope*width
    xmax = (height - intercept)/slope
    a = 0
    if (y0 >= 0 and y0<height):
        points.append([0,math.ceil(y0)])
    if (x0 >= 0 and x0<width and not (x0==0 and y0==0)):
        points.append([math.ceil(x0),0])
    if (ymax>=0 and ymax<=height):
        points.append([width-1,math.ceil(ymax)-1])
    if (xmax >=0 and xmax<=width):
        points.append([math.ceil(xmax)-1,height-1])
        
    if (len(points) < 2):
        return False
        
    cv2.line(img, (points[0][0], points[0][1]), (points[1][0], points[1][1]), (b, g, r), thickness)
        
  


img_arr = cv2.imread('smiley.png', 1)
drawLine(img_arr,10,0,1,0,0,255,4)
cv2.imwrite('color_img2.jpg', img_arr)


# In[237]:


def invert(list):
    return [list[0]] + list[-1:0:-1]

def get_angle(a, b, c):
    angle = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))

   
    return angle + 360 if angle < 0 else angle

def solve(points):
    n = len(points)
    for i in range(len(points)):
        p1 = points[i-2]
        p2 = points[i-1]
        p3 = points[i]
        if get_angle(p1, p2, p3) > 180:
            return True
    return False


# In[243]:


def cropImage(img, p1,p2,p3,p4):
    # p1 = [24,33]
    vab = [p2[0]-p1[0], p2[1]-p1[1]]
    vbc = [p3[0]-p2[0], p3[1]-p2[1]]
    vcd = [p4[0]-p3[0], p4[1]-p3[1]]
    vda = [p1[0]-p4[0], p1[1]-p4[1]]
    

    points = [p1,p2,p3,p4]


    if (np.cross(vab,vbc)/abs(np.cross(vab,vbc))* np.cross(vcd,vda)/abs(np.cross(vab,vbc)) < 0):
        print(np.cross(vab,vbc)* np.cross(vcd,vda))
        points = [p1,p3,p2,p4]
        
    if (solve(points) == True and solve(invert(points)) == True):
        print("It is concave")
        return False

        
    
    x0 = (points[0][0] + points[2][0])/2
    y0 = (points[0][1] + points[2][1])/2
    
    cropLine(img_arr,x0,y0,points[0][0],points[0][1],points[1][0],points[1][1])
    cropLine(img_arr,x0,y0,points[1][0],points[1][1],points[2][0],points[2][1])
    cropLine(img_arr,x0,y0,points[2][0],points[2][1],points[3][0],points[3][1])
    cropLine(img_arr,x0,y0,points[3][0],points[3][1],points[0][0],points[0][1])
    
    
def cropLine(img,x0,y0,x1,y1,x2,y2):
    if ((x2-x1) == 0):
        slope = 1000
    else:
        slope = (y2-y1)/(x2-x1)
    
    intercept = y1 - slope*x1
    

    
    p0 = y0 - slope*x0 - intercept
    
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            p1 = i - slope*j - intercept
            if (p1*p0 < 0):
                img[i,j] = [0,0,0]
        




img_arr = cv2.imread('smiley.png', 1)
cropImage(img_arr, [10,10],[270,100],[150,100],[10,200])
cv2.imwrite('color_img2.jpg', img_arr)


