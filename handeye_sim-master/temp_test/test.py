import os
def getfile(root_dir):
    files = os.listdir(root_dir)
    #print(root_dir)
    for file in files:
        path = os.path.join(root_dir,file)
        if os.path.isdir(path):
            getfile(path)
        else:
            if path.endswith(".urdf") or path.endswith(".launch") or path.endswith(".xacro") :
                #print(path)
                with open(path,'r') as f:
                    str = f.read()
                    if not str.find("pencil")==-1:
                        print(str)
                        print(path)
getfile("/home/speedbot/code/aubo_shanghai_calibration/src/aubo_robot")