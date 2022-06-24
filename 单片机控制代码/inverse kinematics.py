import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__' :
  # 定义结构参数
  b = 104
  w = 191.8
  l = 245
  # 定义目标位姿
  pos = np.mat([0,  0,  110 ]).T # 目标位置向量
  rpy = np.array([0,  0,  15]) * math.pi / 180 # 欧拉角，化为弧度值
  # 将欧拉角转换为旋转矩阵
  R, P, Y = rpy[0], rpy[1], rpy[2]
  rotx = np.mat([[ 1,       0,            0          ],
                 [ 0,       math.cos(R), -math.sin(R)],
                 [ 0,       math.sin(R),  math.cos(R)]])
  roty = np.mat([[ math.cos(P),  0,      -math.sin(P)],
                 [ 0,            1,       0          ],
                 [ math.sin(P),  0,       math.cos(P)]]) 
  rotz = np.mat([[ math.cos(Y), -math.sin(Y),  0     ],
                 [ math.sin(Y),  math.cos(Y),  0     ],
                 [ 0,            0,            1     ]])
  rot_mat = rotx * roty * rotz
  # 结构参数
  body_struc = np.mat([[ l / 2,  b / 2,  0],
                       [ l / 2, -b / 2,  0],
                       [-l / 2, -b / 2,  0],
                       [-l / 2,  b / 2,  0]]).T
  footpoint_struc = np.mat([[ l / 2,  w / 2,  0],
                            [ l / 2, -w / 2,  0],
                            [-l / 2, -w / 2,  0],
                            [-l / 2,  w / 2,  0]]).T
  # 计算单腿末端位置向量AB
  print(rot_mat)
  AB = np.mat(np.zeros((3, 4)))
  for i in range(4):
    AB[:, i] = - pos - rot_mat * body_struc[:, i] + footpoint_struc[:, i]
  print(AB)
  #身体四点
  x1=-AB[0,:]+footpoint_struc[0,:]
  y1=-AB[1,:]+footpoint_struc[1,:]
  z1=-AB[2,:]+footpoint_struc[2,:]
  #足四点
  x2=footpoint_struc[0,:]
  y2=footpoint_struc[1,:]
  z2=footpoint_struc[2,:]
  
  fig = plt.figure()
  ax = Axes3D(fig)
  #ax.scatter(x1, y1, z1)
  #ax.scatter(x2, y2, z2)
  for i in range(4):
    if i==3:
      x=[x1[0,3],x1[0,0]]
      y=[y1[0,3],y1[0,0]]
      z=[z1[0,3],z1[0,0]]
    else:
      x=[x1[0,i],x1[0,i+1]]
      y=[y1[0,i],y1[0,i+1]]
      z=[z1[0,i],z1[0,i+1]]
    ax.plot(x, y, z, c='r')
    
    x=[x1[0,i],x2[0,i]]
    y=[y1[0,i],y2[0,i]]
    z=[z1[0,i],z2[0,i]]
    ax.plot(x, y, z, c='b')

    
  ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
  ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
  ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
  plt.show()
