import math
import numpy as np
import sys
from coordinates.converter import CoordinateConverter, WGS84, L_Est97
converter = CoordinateConverter

#W=float(input('enter view'))
#H=input('enter height(in m)')
#V=int(input('enter distance out of fence'))

orig_stdout = sys.stdout
def main(FENCE,H,W,V):
    f=open('mission_path_1.plan',"w")
    sys.stdout = f


    wgs_point1 = WGS84(lat=FENCE[0][0],long=FENCE[0][1])
    wgs_point2 = WGS84(lat=FENCE[1][0],long=FENCE[1][1])
    wgs_point3 = WGS84(lat=FENCE[2][0],long=FENCE[2][1])
    wgs_point4 = WGS84(lat=FENCE[3][0],long=FENCE[3][1])
    X1,Y1=converter.wgs84_to_l_est97(wgs_point1)
    X2,Y2=converter.wgs84_to_l_est97(wgs_point2)
    X3,Y3=converter.wgs84_to_l_est97(wgs_point3)
    X4,Y4=converter.wgs84_to_l_est97(wgs_point4)

    X=math.sqrt((X1-X2)**2+(Y1-Y2)**2)
    Y=math.sqrt((X1-X4)**2+(Y1-Y4)**2)


    B1=[]
    B2=[]

    r=X%W

    if 0<r<1:
      p=int(X/W)
      m=1
      n=2*p-1
      for j in range(p):
         x1=(n*X1+m*X2)/(2*p)
         y1=(n*Y1+m*Y2)/(2*p)
         B1.append([x1,y1])
         x2=(n*X4+m*X3)/(2*p)
         y2=(n*Y4+m*Y3)/(2*p)
         B2.append([x2,y2])
         m=m+2
         n=n-2

    if r>1:
       p=int((X-r)/W)
       s=r/2
       m1=s
       n1=X-s
       [P11,Q11]=[ (n1*X1+m1*X2)/(m1+n1),(n1*Y1+m1*Y2)/(m1+n1)]
       [P12,Q12]=[ (m1*X1+n1*X2)/(m1+n1),(m1*Y1+n1*Y2)/(m1+n1)]
       [P21,Q21]=[ (n1*X4+m1*X3)/(m1+n1),(n1*Y4+m1*Y3)/(m1+n1)]
       [P22,Q22]=[ (m1*X4+n1*X3)/(m1+n1),(m1*Y4+n1*Y3)/(m1+n1)]
       B1.append([P11,Q11])
       B1.append([P12,Q12])
       B2.append([P21,Q21])
       B2.append([P22,Q22])
       m=1
       n=2*p-1
       for j in range(p):
            x1=(n*P11+m*P12)/(2*p)
            y1=(n*Q11+m*Q12)/(2*p)
            B1.append([x1,y1])
            x2=(n*P21+m*P22)/(2*p)
            y2=(n*Q21+m*Q22)/(2*p)
            B2.append([x2,y2])
            m=m+2
            n=n-2

    C1=[]
    C2=[]

    def contours(C1,C2,q,B1,B2,Y,V):

        j=0
        for j in range(q):
            m=V
            n=Y+V
            k1=(B1[0:q][j][0])
            l1=(B1[0:q][j][1])
            k2=(B2[0:q][j][0])
            l2=(B2[0:q][j][1])
            k11 = (n*k1-k2*m)/Y
            l11 = (n*l1-l2*m)/Y
            C1.append([k11,l11])

        for j in range(q):
            m=V
            n=Y+V
            k1=(B1[0:q][j][0])
            l1=(B1[0:q][j][1])
            k2=(B2[0:q][j][0])
            l2=(B2[0:q][j][1])
            k12 = (n*k2-k1*m)/Y
            l12 = (n*l2-l1*m)/Y
            C2.append([k12,l12])

    if 0<r<1:
        q=p
    else:
        q=p+2
    contours(C1,C2,q,B1,B2,Y,V)

    PATH=[]

    for i in range(q):

        if (i%2==0):
            PATH.append(C1[i])
            PATH.append(B1[i])
            PATH.append(B2[i])
            PATH.append(C2[i])
        else:
            PATH.append(C2[i])
            PATH.append(B2[i])
            PATH.append(B1[i])
            PATH.append(C1[i])

    num=len(PATH)
    WAY=[]
    for k in range(num):
        e1 = L_Est97(x=PATH[k][0], y=PATH[k][1])
        T1,T2=converter.l_est97_to_wgs84(e1)
        WAY.append([T2,T1])

    print('{\n\
                    "fileType":"Plan",\n\
                    "geoFence": {\n\
                        "circles":[\n\
                        ],\n\
                        "polygons":[\n\
                        ],\n\
                        "version" :2\n\
                    },\n\
                    "groundStation" :"QGroundControl",\n\
                    "mission" : {\n\
                        "cruiseSpeed" :15,\n\
                        "firmwareType" :12,\n\
                        "hoverSpeed" : 5,\n\
                        "items":[\n\
                                {\n\
                                     "autoContinue": true,\n\
                                     "command": 178,\n\
                                     "doJumpId": \n',1, ',\n'
                                   ' "frame": 2,\n\
                                     "params":[\n\
                                         1,\n\
                                         1,\n\
                                        -1,\n\
                                         0,\n'
    '                                    0,\n'
    '                                    0,\n'
    '                                    0\n'
                                      '],\n\
                                      "type": "SimpleItem"\n\
                                    },\n\
                            { \n\
                                "TransectStyleComplexItem" : {\n\
                                    "CameraCalc" : {\n\
                                        "AdjustedFootprintFrontal": 25,\n\
                                        "AdjustedFootprintSide": 10,\n\
                                        "CameraName" : "Manual (no camera specs)",\n\
                                        "DistanceToSurface" : 50,\n\
                                        "DistanceToSurfaceRelative" : true,\n\
                                        "version" : 1\n\
                                    },\n\
                                    "CameraShots" : 30,\n\
                                    "CameraTriggerInTurnAround"   : true,\n\
                                    "FollowTerrain": false,\n\
                                    "HoverAndCapture":false,\n\
                                    "Items" : [\n')

    for k in range (num):
        if k==num-1:
            print('                                    {\n\
                                                          "autoContinue": true,\n\
                                                          "command": 16,\n\
                                                          "doJumpId":  \n',2+k, ',\n'
                                                         ' "frame": 3,\n\
                                                          "params":[\n\
                                                              0,\n\
                                                              0,\n\
                                                              0,\n\
                                                              null,\n'
            '                                                  ',WAY[k][0],',\n'
            '                                                 ' ,WAY[k][1],',\n'
            '                                                 ' ,H,'\n'
                                                          '],\n\
                                                         "type": "SimpleItem"\n\
                                                       }\n' )

        else:
            print('                                    {\n\
                                                          "autoContinue": true,\n\
                                                          "command": 16,\n\
                                                          "doJumpId": \n',2+k, ',\n'
                                                          '"frame": 3,\n\
                                                          "params":[\n\
                                                              0,\n\
                                                              0,\n\
                                                              0,\n\
                                                              null,\n'
            '                                                 ' ,WAY[k][0],',\n'
            '                                                 ' ,WAY[k][1],',\n'
            '                                                 ' ,H,'\n'
                                                          '],\n\
                                                         "type": "SimpleItem"\n\
                                                       },\n' )
        k=k+1





    print('                                  ],\n\
                                             "Refly90Degrees": false,\n\
                                             "TurnAroundDistance": 10,\n\
                                             "VisualTransectPoints": [\n')

    for j in range(num):
        if j==num-1:
            print('                                  [\n'
                '                                    ',WAY[j][0],',\n'
                '                                    ' ,WAY[j][1],'\n'
                                                    ']\n')
        else:
            print('                                  [\n'
                '                                        ',WAY[j][0],',\n'
                '                                        ',WAY[j][1],'\n'
                                                    '],\n')



    print('                                    ],\n\
                                                   "version": 1\n\
                                             },\n\
                                             "angle": 0,\n\
                                             "complexItemType": "survey",\n\
                                             "entryLocation": 0,\n\
                                             "flyAlternateTransects": false,\n\
                                             "polygon": [\n\
                                                 \n')
    for j in range(4):
        if(j==3):
            print('                                           [\n'
                    '                                         ',FENCE[j][0],',\n'
                    '                                         ',FENCE[j][1],'\n'
                                                      ']\n')
        else:
            print('                                           [\n'
                    '                                        ' ,FENCE[j][0],',\n'
                    '                                        ' ,FENCE[j][1],'\n'
                                                      '],\n')




    print('                             ],\n\
                                       "splitConcavePolygons": false,\n\
                                       "type": "ComplexItem",\n\
                                       "version": 5\n\
                                  }\n\
                          ],\n\
                          "plannedHomePosition": [\n'

    '                             ',WAY[0][0],',\n'
    '                             ',WAY[0][1],',\n',
    '                             ',H,


                         '],\n\
                         "vehicleType": 2,\n\
                         "version": 2\n\
                    },\n\
                    "rallyPoints": {\n\
                        "points": [\n\
                         ],\n\
                         "version": 2\n\
                    },\n\
                   "version": 1\n\
               }\n')
    sys.stdout = orig_stdout
    f.close()
