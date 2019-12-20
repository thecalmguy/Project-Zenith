import math
import numpy as np
import sys
import os
import plan_1
import plan_2
import plan_3

from coordinates.converter import CoordinateConverter, WGS84, L_Est97
converter = CoordinateConverter

W=float(input('enter view'))
H=input('enter height(in m)')
V=int(input('enter distance out of fence'))

orig_stdout = sys.stdout
#def main():
#f=open('mission_path.plan',"w")
#sys.stdout = f
F1=[]
F2=[]
F3=[]
F4=[]

#print (enter 4 GPS coordinates)
'''for i in range(8):
    ord=float(input())
    if i<=1:
        F1.append(ord)
    if 1<i<=3:
        F2.append(ord)
    if 3<i<=5:
        F3.append(ord)
    if 5<i<8:
        F4.append(ord)'''
F1=[26.1934511,91.6970978]
F2=[26.1934511,91.6991051]
F3=[26.1925433,91.6991051]
F4=[26.1925433,91.6970978]
#print(F1,F2,F3,F4)
add_fence_points=[]
add_fence_points.append(F1)
add_fence_points.append(F2)
add_fence_points.append(F3)
add_fence_points.append(F4)
#print(add_fence_points)
wgs_point1 = WGS84(lat=F1[0],long=F1[1])
wgs_point2 = WGS84(lat=F2[0],long=F2[1])
wgs_point3 = WGS84(lat=F3[0],long=F3[1])
wgs_point4 = WGS84(lat=F4[0],long=F4[1])
X1,Y1=converter.wgs84_to_l_est97(wgs_point1)
X2,Y2=converter.wgs84_to_l_est97(wgs_point2)
X3,Y3=converter.wgs84_to_l_est97(wgs_point3)
X4,Y4=converter.wgs84_to_l_est97(wgs_point4)

X=math.sqrt((X1-X2)**2+(Y1-Y2)**2)
Y=math.sqrt((X1-X4)**2+(Y1-Y4)**2)

a=1
b=2

X11=(X1*b+X2*a)/(a+b)
Y11=(Y1*b+Y2*a)/(a+b)

X22=(X1*a+X2*b)/(a+b)
Y22=(Y1*a+Y2*b)/(a+b)

X44=(X4*b+X3*a)/(a+b)
Y44=(Y4*b+Y3*a)/(a+b)

X33=(X4*a+X3*b)/(a+b)
Y33=(Y4*a+Y3*b)/(a+b)

add_fence=[]
add_fence.append([X11,Y11])
add_fence.append([X22,Y22])
add_fence.append([X33,Y33])
add_fence.append([X44,Y44])
F=[]
for k in range(4):
    e1 = L_Est97(x=add_fence[k][0], y=add_fence[k][1])
    T1,T2=converter.l_est97_to_wgs84(e1)
    F.append([T2,T1])

fence_1=[F1,F[0],F[3],F4]
fence_2=F
fence_3=[F[1],F2,F3,F[2]]

plan_1.main(fence_1,H,W,V)
plan_2.main(fence_2,H,W,V)
plan_3.main(fence_3,H,W,V)
