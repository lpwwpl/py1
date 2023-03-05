# _*_ coding:utf-8 _*_
# @time: 2020/9/26 上午10:03
# @author: 张新新
# @email: 1262981714@qq.com
from Vrep import vrep
import time
def test():
    print('Program started')
    # 关闭潜在的连接
    vrep.simxFinish(-1)
    # 每隔0.2s检测一次，直到连接上V-rep
    while True:
        clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if clientID > -1:
            break
        else:
            time.sleep(0.2)
            print("Failed connecting to remote API server!")
    print("Connection success!")
    errorCode, dummyHandle = vrep.simxGetObjectHandle(clientID, 'Dummy_target', vrep.simx_opmode_oneshot_wait)
    print(errorCode)
    errorCode, position = vrep.simxGetObjectPosition(clientID,dummyHandle,-1,vrep.simx_opmode_oneshot_wait)
    print(position)
    vrep.simxSetObjectPosition(clientID,dummyHandle,-1,[-0.2,0,0.3],vrep.simx_opmode_oneshot_wait)
test()