# _*_ coding:utf-8 _*_
# @time: 2020/9/28 上午10:25
# @author: 张新新
# @email: 1262981714@qq.com
from Vrep import vrep
import time
def getVrep_connect(port):
    vrep.simxFinish(-1)
    print("attempt to connect vrep")
    while True:
        clientID = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)
        if clientID > -1:
            break
        else:
            time.sleep(0.2)
    print("get connection to vrep")
    return clientID