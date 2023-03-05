import os
import shutil
def getImgList(root_dir):
    file_list = os.listdir(root_dir)
    rgb_list = []
    depth_list = []
    for file in file_list:
        if file.endswith("_color.bmp"):
            rgb_list.append(os.path.join(root_dir,file))
        elif file.endswith("_depth.png"):
            depth_list.append(os.path.join(root_dir,file))
    return rgb_list,depth_list

color,depth = getImgList("../real_data/2020_12_15_20_08")
save_dir = "../real_data/intrinisc_data"
color.sort()
depth.sort()
i=30
j=0
for j in range(len(color)):
    shutil.copy(color[j], os.path.join(save_dir,"{}_color.bmp".format(i+j)))
    shutil.copy(depth[j], os.path.join(save_dir,"{}_depth.png".format(i+j)))