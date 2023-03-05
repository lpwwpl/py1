# _*_ coding:utf-8 _*_
# @time: 2020/10/8 下午7:50
# @author: 张新新
# @email: 1262981714@qq.com
from Vrep import LBR4p
from Vrep import vrep_connect
id = vrep_connect.getVrep_connect()
robot = LBR4p.Robot(id)
pose = robot.getPoseMatrix()
pose[2,3] = 0.725
print(robot.move(pose))
z_max = pose[2,3]
# z_min = 0
# while True:
#     z_mid = (z_max+z_min)/2
#     pose[2,3] = z_mid
#     flag = robot.move(pose)
#     if flag:
#         z_max = z_mid
#     else:
#         z_min = z_mid
#     if z_max-z_mid<0.01:
#         print(z_max,z_min)
#     print(z_max,z_min)


