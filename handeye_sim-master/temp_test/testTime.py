import time

timeArray = time.localtime()
otherStyleTime = time.strftime("%Y_%m_%d_%H_%M", timeArray)
print(otherStyleTime)