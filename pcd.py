# import Image as img
import kdata_lx
import numpy as np
import matplotlib
import os
import sys
import cv2 as cv

def meshwrite(filename, verts, faces, norms, colors):
  """Save a 3D mesh to a polygon .ply file.
  """
  # Write header
  ply_file = open(filename,'w')
  ply_file.write("ply\n")
  ply_file.write("format ascii 1.0\n")
  ply_file.write("element vertex %d\n"%(verts.shape[0]))
  ply_file.write("property float x\n")
  ply_file.write("property float y\n")
  ply_file.write("property float z\n")
  ply_file.write("property float nx\n")
  ply_file.write("property float ny\n")
  ply_file.write("property float nz\n")
  ply_file.write("property uchar red\n")
  ply_file.write("property uchar green\n")
  ply_file.write("property uchar blue\n")
  ply_file.write("element face %d\n"%(faces.shape[0]))
  ply_file.write("property list uchar int vertex_index\n")
  ply_file.write("end_header\n")

def plywrite(filename, xyzrgb):
  """Save a point cloud to a polygon .ply file.
  """
  xyz = xyzrgb[:, :3]
  rgb = xyzrgb[:, 3:].astype(np.uint8)

  # Write header
  ply_file = open(filename,'w')
  ply_file.write("ply\n")
  ply_file.write("format ascii 1.0\n")
  ply_file.write("element vertex %d\n"%(xyz.shape[0]))
  ply_file.write("property float x\n")
  ply_file.write("property float y\n")
  ply_file.write("property float z\n")
  ply_file.write("property uchar red\n")
  ply_file.write("property uchar green\n")
  ply_file.write("property uchar blue\n")
  ply_file.write("end_header\n")

  # Write vertex list
  for i in range(xyz.shape[0]):
    ply_file.write("%f %f %f %d %d %d\n"%(
      xyz[i, 0], xyz[i, 1], xyz[i, 2],
      rgb[i, 0], rgb[i, 1], rgb[i, 2],
    ))


def pcdwrite(filename, xyzh):
  """Save a point cloud to a polygon .ply file.
  """
  xyz = xyzh[:, :3]
  rgb = xyzh[:, 3:]#.astype(np.uint8)

  # Write header
  ply_file = open(filename,'w')
  ply_file.write("VERSION .7\n")
  ply_file.write("FIELDS x y z rgb\n")
  ply_file.write("SIZE 4 4 4 4\n")
  ply_file.write("TYPE F F F F\n")
  ply_file.write("COUNT 1 1 1 1\n")
  ply_file.write("WIDTH %d\n"%(xyz.shape[0]))
  ply_file.write("HEIGHT 1\n")
  ply_file.write("VIEWPOINT 0 0 0 1 0 0 0\n")
  ply_file.write("POINTS %d\n"%(xyz.shape[0]))
  ply_file.write("DATA ascii\n")

  # Write vertex list
  for i in range(xyz.shape[0]):
    ply_file.write("%f %f %f %d\n"%(
      xyz[i, 0], xyz[i, 1], xyz[i, 2], rgb[i, 0]
    ))

# def get_mesh(self):
#     """Compute a mesh from the voxel volume using marching cubes.
#     """
#     tsdf_vol, color_vol = self.get_volume()
#
#     # Marching cubes
#     verts, faces, norms, vals = measure.marching_cubes_lewiner(tsdf_vol, level=0)
#     verts_ind = np.round(verts).astype(int)
#     verts = verts * self._voxel_size + self._vol_origin  # voxel grid coordinates to world coordinates
#
#     # Get vertex colors
#     rgb_vals = color_vol[verts_ind[:, 0], verts_ind[:, 1], verts_ind[:, 2]]
#     colors_b = np.floor(rgb_vals / self._color_const)
#     colors_g = np.floor((rgb_vals - colors_b * self._color_const) / 256)
#     colors_r = rgb_vals - colors_b * self._color_const - colors_g * 256
#     colors = np.floor(np.asarray([colors_r, colors_g, colors_b])).T
#     colors = colors.astype(np.uint8)
#     return verts, faces, norms, colors

def executable_path():
    return os.path.dirname(sys.argv[0])

def calc_object_2_base():
    pass

def outputPLY():
    ret = []
    try:
        n_imgs = 1
        cam_intr = np.loadtxt("data/camera-intrinsics.txt", delimiter=' ')
        for i in range(n_imgs):
            exec_path = executable_path()
            #intriFileName = os.path.abspath(os.path.join(exec_path, "picslx/test-camera-intrinsics.txt"))
            #kdata_lx.camIntrinsics = np.loadtxt(intriFileName, dtype=float)

            #prefix = 'frame-20210303_175313'
            exec_path = executable_path()
            colorFileName = os.path.abspath(os.path.join(exec_path, "data/frame-%06d.color.jpg"%(i)))
            deptFileName = os.path.abspath(os.path.join(exec_path, "data/frame-%06d.depth.png"%(i)))
            #inferFileName = os.path.abspath(os.path.join(exec_path, "data/{}.infer.png".format(prefix)))
            color_img = cv.cvtColor(cv.imread(colorFileName, flags=1), cv.COLOR_BGR2RGB)
            depth_img = cv.imread(deptFileName, flags=-1)
            #inferImg = cv.imread(inferFileName, flags=-1)

            iheight = 480
            iwidth = 640
            kdata_lx.depthImg = depth_img
            #kdata_lx.inferImg = inferImg
            kdata_lx.colorImg = color_img

            depthImg = kdata_lx.depthImg.astype('double')/10000.0

            affordance_threthold = kdata_lx.inferImg.max().astype('double')/255.0*0.63
            affordance_threthold = 0

            mask_th = 255 * affordance_threthold
            mask_p = (kdata_lx.inferImg >= mask_th)

            x = np.arange(1, iwidth + 1)
            y = np.arange(1, iheight + 1)
            [pixX, pixY] = np.meshgrid(x, y)
            pixZ = depthImg.astype(np.double)

            pixX = pixX.T
            pixX_clusters = pixX#pixX[mask_p.T]

            pixY = pixY.T
            pixY_clusters = pixY#[mask_p.T]

            pixZ = pixZ.T
            pixZ_clusters = pixZ#[mask_p.T]

            pixels_clusters = np.asarray([pixX_clusters, pixY_clusters])

            camX_clusters = (pixX_clusters - cam_intr[0][2]) * pixZ_clusters / cam_intr[0][0]
            camY_clusters = (pixY_clusters - cam_intr[1][2]) * pixZ_clusters / cam_intr[1][1]
            camZ_clusters = pixZ_clusters

            camPoints_clusters = np.asarray([camX_clusters, camY_clusters, camZ_clusters])
            camPoints_clusters_flat = camPoints_clusters.reshape(-1,3)
            posX = []
            posY = []
            posZ = []
            colors = []
            # img_file = img.open(colorFileName)
            cam_pose = np.loadtxt("data/frame-%06d.pose.txt" % (i))
            cam_pts = rigid_transform(camPoints_clusters_flat, np.linalg.inv(cam_pose))
            for l in range(0,camX_clusters.size):
                row = int(l/480)
                column = l - int(row*480)
                c1 = kdata_lx.colorImg[column,row]
                # c1 = kdata_lx.colorImg[column,row]
                # print(c1)
                ret.append([camX_clusters[l], camY_clusters[l], camZ_clusters[l], c1[0], c1[1], c1[2]])
                # ret.append([cam_pts[l][0], cam_pts[l][1], cam_pts[l][2], c1[0], c1[1], c1[2]])
            #fusion.pcwrite("pc.ply", point_cloud)
            # for l in range(0, camX_clusters.size):
            #     if(camZ_clusters[l] > 0):
            #         # pos = calc_object_2_base([camX_clusters[l]*1000, camY_clusters[l]*1000, camZ_clusters[l]*1000, 0,0,0])
            #         # posX.append(camX_clusters[0]) #*0.001
            #         # posY.append(camX_clusters[1]) #*0.001
            #         # posZ.append(camX_clusters[2]) #*0.001
            #         c1 = kdata_lx.colorImg[pixY[l], pixX[l]]
            #         #colors.append(matplotlib.colors.to_hex([c1.r, c1.g, c1.b]))
            #         ret.append([posX,posY,posZ,c1.r,c1.g,c1.b])
    except Exception as e:
        print(e)

    array_ret = np.asarray(ret)
    return array_ret

def rigid_transform(xyz, transform):
  """Applies a rigid transform to an (N, 3) pointcloud.
  """
  xyz_h = np.hstack([xyz, np.ones((len(xyz), 1), dtype=np.float32)])
  xyz_t_h = np.dot(transform, xyz_h.T).T
  return xyz_t_h[:, :3]

def rgb2hex(rgb):
    result = rgb[0]<<16 | (rgb[1] << 8) | (rgb[2])
    return result

def outputPCD():
    ret = []
    try:
        exec_path = executable_path()
        intriFileName = os.path.abspath(os.path.join(exec_path, "picslx/test-camera-intrinsics.txt"))
        kdata_lx.camIntrinsics = np.loadtxt(intriFileName, dtype=float)

        prefix = 'frame-20210303_175313'
        exec_path = executable_path()
        colorFileName = os.path.abspath(os.path.join(exec_path, "picslx/{}.color.png".format(prefix)))
        deptFileName = os.path.abspath(os.path.join(exec_path, "picslx/{}.depth.png".format(prefix)))
        inferFileName = os.path.abspath(os.path.join(exec_path, "picslx/{}.infer.png".format(prefix)))
        color_img = cv.cvtColor(cv.imread(colorFileName, flags=1), cv.COLOR_BGR2RGB)
        depth_img = cv.imread(deptFileName, flags=-1)
        inferImg = cv.imread(inferFileName, flags=-1)

        iheight = 480
        iwidth = 640
        kdata_lx.depthImg = depth_img
        kdata_lx.inferImg = inferImg
        kdata_lx.colorImg = color_img

        depthImg = kdata_lx.depthImg.astype('double')/10000.0

        affordance_threthold = kdata_lx.inferImg.max().astype('double')/255.0*0.63
        affordance_threthold = 0

        mask_th = 255 * affordance_threthold
        mask_p = (kdata_lx.inferImg >= mask_th)

        x = np.arange(1, iwidth + 1)
        y = np.arange(1, iheight + 1)
        [pixX, pixY] = np.meshgrid(x, y)
        pixZ = depthImg.astype(np.double)

        pixX = pixX.T
        pixX_clusters = pixX[mask_p.T]

        pixY = pixY.T
        pixY_clusters = pixY[mask_p.T]

        pixZ = pixZ.T
        pixZ_clusters = pixZ[mask_p.T]

        pixels_clusters = np.asarray([pixX_clusters, pixY_clusters])

        camX_clusters = (pixX_clusters - kdata_lx.camIntrinsics[0][2]) * pixZ_clusters / kdata_lx.camIntrinsics[0][0]
        camY_clusters = (pixY_clusters - kdata_lx.camIntrinsics[1][2]) * pixZ_clusters / kdata_lx.camIntrinsics[1][1]
        camZ_clusters = pixZ_clusters

        camPoints_clusters = np.asarray([camX_clusters, camY_clusters, camZ_clusters])
        posX = []
        posY = []
        posZ = []
        colors = []
        # img_file = img.open(colorFileName)
        for l in range(0,camX_clusters.size):
            row = int(l/480)
            column = l - int(row*480)
            c1 = kdata_lx.colorImg[column,row]
            # c1 = kdata_lx.colorImg[column,row]
            # print(c1)
            ret.append([camX_clusters[l], camY_clusters[l], camZ_clusters[l], rgb2hex(c1)])#rgb2hex(c1)
        #fusion.pcwrite("pc.ply", point_cloud)
        # for l in range(0, camX_clusters.size):
        #     if(camZ_clusters[l] > 0):
        #         # pos = calc_object_2_base([camX_clusters[l]*1000, camY_clusters[l]*1000, camZ_clusters[l]*1000, 0,0,0])
        #         # posX.append(camX_clusters[0]) #*0.001
        #         # posY.append(camX_clusters[1]) #*0.001
        #         # posZ.append(camX_clusters[2]) #*0.001
        #         c1 = kdata_lx.colorImg[pixY[l], pixX[l]]
        #         #colors.append(matplotlib.colors.to_hex([c1.r, c1.g, c1.b]))
        #         ret.append([posX,posY,posZ,c1.r,c1.g,c1.b])
    except Exception as e:
        print(e)

    array_ret = np.asarray(ret)
    return array_ret

if __name__ == '__main__':

    point_cloud = outputPLY()#outputPCD()
    if point_cloud.size > 0:
        plywrite("1.ply",point_cloud)
        # pcdwrite("1.pcd", point_cloud)
    # print("Saving mesh to mesh.ply...")
    # verts, faces, norms, colors = get_mesh()
    # meshwrite("mesh.ply", verts, faces, norms, colors)
    #
    # # Get point cloud from voxel volume and save to disk (can be viewed with Meshlab)
    # print("Saving point cloud to pc.ply...")
    # point_cloud = tsdf_vol.get_point_cloud()
    # pcwrite("pc.ply", point_cloud)

