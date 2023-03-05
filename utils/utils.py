# coding=utf-8
# Copyright 2021 The Ravens Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Miscellaneous utilities."""

import cv2
import matplotlib
import matplotlib.pyplot as plt
import meshcat
import meshcat.geometry as g
import meshcat.transformations as mtf

import numpy as np
from transforms3d import euler

import pybullet as p

#-----------------------------------------------------------------------------
# HEIGHTMAP UTILS
#-----------------------------------------------------------------------------


def get_heightmap(points, colors, bounds, pixel_size):
  """Get top-down (z-axis) orthographic heightmap image from 3D pointcloud.

  Args:
    points: HxWx3 float array of 3D points in world coordinates.
    colors: HxWx3 uint8 array of values in range 0-255 aligned with points.
    bounds: 3x2 float array of values (rows: X,Y,Z; columns: min,max) defining
      region in 3D space to generate heightmap in world coordinates.
    pixel_size: float defining size of each pixel in meters.

  Returns:
    heightmap: HxW float array of height (from lower z-bound) in meters.
    colormap: HxWx3 uint8 array of backprojected color aligned with heightmap.
  """
  width = int(np.round((bounds[0, 1] - bounds[0, 0]) / pixel_size))
  height = int(np.round((bounds[1, 1] - bounds[1, 0]) / pixel_size))
  heightmap = np.zeros((height, width), dtype=np.float32)
  colormap = np.zeros((height, width, colors.shape[-1]), dtype=np.uint8)

  # Filter out 3D points that are outside of the predefined bounds.
  ix = (points[Ellipsis, 0] >= bounds[0, 0]) & (points[Ellipsis, 0] < bounds[0, 1])
  iy = (points[Ellipsis, 1] >= bounds[1, 0]) & (points[Ellipsis, 1] < bounds[1, 1])
  iz = (points[Ellipsis, 2] >= bounds[2, 0]) & (points[Ellipsis, 2] < bounds[2, 1])
  valid = ix & iy & iz
  points = points[valid]
  colors = colors[valid]

  # Sort 3D points by z-value, which works with array assignment to simulate
  # z-buffering for rendering the heightmap image.
  iz = np.argsort(points[:, -1])
  points, colors = points[iz], colors[iz]
  px = np.int32(np.floor((points[:, 0] - bounds[0, 0]) / pixel_size))
  py = np.int32(np.floor((points[:, 1] - bounds[1, 0]) / pixel_size))
  px = np.clip(px, 0, width - 1)
  py = np.clip(py, 0, height - 1)
  heightmap[py, px] = points[:, 2] - bounds[2, 0]
  for c in range(colors.shape[-1]):
    colormap[py, px, c] = colors[:, c]
  return heightmap, colormap


def get_pointcloud(depth, intrinsics):
  """Get 3D pointcloud from perspective depth image.

  Args:
    depth: HxW float array of perspective depth in meters.
    intrinsics: 3x3 float array of camera intrinsics matrix.

  Returns:
    points: HxWx3 float array of 3D points in camera coordinates.
  """
  height, width = depth.shape
  xlin = np.linspace(0, width - 1, width)
  ylin = np.linspace(0, height - 1, height)
  px, py = np.meshgrid(xlin, ylin)
  px = (px - intrinsics[0, 2]) * (depth / intrinsics[0, 0])
  py = (py - intrinsics[1, 2]) * (depth / intrinsics[1, 1])
  points = np.float32([px, py, depth]).transpose(1, 2, 0)
  return points


def transform_pointcloud(points, transform):
  """Apply rigid transformation to 3D pointcloud.

  Args:
    points: HxWx3 float array of 3D points in camera coordinates.
    transform: 4x4 float array representing a rigid transformation matrix.

  Returns:
    points: HxWx3 float array of transformed 3D points.
  """
  padding = ((0, 0), (0, 0), (0, 1))
  homogen_points = np.pad(points.copy(), padding,
                          'constant', constant_values=1)
  for i in range(3):
    points[Ellipsis, i] = np.sum(transform[i, :] * homogen_points, axis=-1)
  return points


def reconstruct_heightmaps(color, depth, configs, bounds, pixel_size):
  """Reconstruct top-down heightmap views from multiple 3D pointclouds."""
  heightmaps, colormaps = [], []
  for color, depth, config in zip(color, depth, configs):
    intrinsics = np.array(config['intrinsics']).reshape(3, 3)
    xyz = get_pointcloud(depth, intrinsics)
    position = np.array(config['position']).reshape(3, 1)
    rotation = p.getMatrixFromQuaternion(config['rotation'])
    rotation = np.array(rotation).reshape(3, 3)
    transform = np.eye(4)
    transform[:3, :] = np.hstack((rotation, position))
    xyz = transform_pointcloud(xyz, transform)
    heightmap, colormap = get_heightmap(xyz, color, bounds, pixel_size)
    heightmaps.append(heightmap)
    colormaps.append(colormap)
  return heightmaps, colormaps


def pix_to_xyz(pixel, height, bounds, pixel_size, skip_height=False):
  """Convert from pixel location on heightmap to 3D position."""
  u, v = pixel
  x = bounds[0, 0] + v * pixel_size
  y = bounds[1, 0] + u * pixel_size
  if not skip_height:
    z = bounds[2, 0] + height[u, v]
  else:
    z = 0.0
  return (x, y, z)


def xyz_to_pix(position, bounds, pixel_size):
  """Convert from 3D position to pixel location on heightmap."""
  u = int(np.round((position[1] - bounds[1, 0]) / pixel_size))
  v = int(np.round((position[0] - bounds[0, 0]) / pixel_size))
  return (u, v)


def unproject_vectorized(uv_coordinates, depth_values,
                         intrinsic,
                         distortion):
  """Vectorized version of unproject(), for N points.

  Args:
    uv_coordinates: pixel coordinates to unproject of shape (n, 2).
    depth_values: depth values corresponding index-wise to the uv_coordinates of
      shape (n).
    intrinsic: array of shape (3, 3). This is typically the return value of
      intrinsics_to_matrix.
    distortion: camera distortion parameters of shape (5,).

  Returns:
    xyz coordinates in camera frame of shape (n, 3).
  """
  cam_mtx = intrinsic  # shape [3, 3]
  cam_dist = np.array(distortion)  # shape [5]

  # shape of points_undistorted is [N, 2] after the squeeze().
  points_undistorted = cv2.undistortPoints(
      uv_coordinates.reshape((-1, 1, 2)), cam_mtx, cam_dist).squeeze()

  x = points_undistorted[:, 0] * depth_values
  y = points_undistorted[:, 1] * depth_values

  xyz = np.vstack((x, y, depth_values)).T
  return xyz


def unproject_depth_vectorized(im_depth, depth_dist,
                               camera_mtx,
                               camera_dist):
  """Unproject depth image into 3D point cloud, using calibration.

  Args:
    im_depth: raw depth image, pre-calibration of shape (height, width).
    depth_dist: depth distortion parameters of shape (8,)
    camera_mtx: intrinsics matrix of shape (3, 3). This is typically the return
      value of intrinsics_to_matrix.
    camera_dist: camera distortion parameters shape (5,).

  Returns:
    numpy array of shape [3, H*W]. each column is xyz coordinates
  """
  h, w = im_depth.shape

  # shape of each u_map, v_map is [H, W].
  u_map, v_map = np.meshgrid(np.linspace(
      0, w - 1, w), np.linspace(0, h - 1, h))

  adjusted_depth = depth_dist[0] + im_depth * depth_dist[1]

  # shape after stack is [N, 2], where N = H * W.
  uv_coordinates = np.stack((u_map.reshape(-1), v_map.reshape(-1)), axis=-1)

  return unproject_vectorized(uv_coordinates, adjusted_depth.reshape(-1),
                              camera_mtx, camera_dist)


#-----------------------------------------------------------------------------
# MATH UTILS
#-----------------------------------------------------------------------------


def sample_distribution(prob, n_samples=1):
  """Sample data point from a custom distribution."""
  flat_prob = prob.flatten() / np.sum(prob)
  rand_ind = np.random.choice(
      np.arange(len(flat_prob)), n_samples, p=flat_prob, replace=False)
  rand_ind_coords = np.array(np.unravel_index(rand_ind, prob.shape)).T
  return np.int32(rand_ind_coords.squeeze())


#-------------------------------------------------------------------------
# Transformation Helper Functions
#-------------------------------------------------------------------------


def invert(pose):
  return p.invertTransform(pose[0], pose[1])


def multiply(pose0, pose1):
  return p.multiplyTransforms(pose0[0], pose0[1], pose1[0], pose1[1])


def apply(pose, position):
  position = np.float32(position)
  position_shape = position.shape
  position = np.float32(position).reshape(3, -1)
  rotation = np.float32(p.getMatrixFromQuaternion(pose[1])).reshape(3, 3)
  translation = np.float32(pose[0]).reshape(3, 1)
  position = rotation @ position + translation
  return tuple(position.reshape(position_shape))


def eulerXYZ_to_quatXYZW(rotation):  # pylint: disable=invalid-name
  """Abstraction for converting from a 3-parameter rotation to quaterion.

  This will help us easily switch which rotation parameterization we use.
  Quaternion should be in xyzw order for pybullet.

  Args:
    rotation: a 3-parameter rotation, in xyz order tuple of 3 floats

  Returns:
    quaternion, in xyzw order, tuple of 4 floats
  """
  euler_zxy = (rotation[2], rotation[0], rotation[1])
  quaternion_wxyz = euler.euler2quat(*euler_zxy, axes='szxy')
  q = quaternion_wxyz
  quaternion_xyzw = (q[1], q[2], q[3], q[0])
  return quaternion_xyzw


def quatXYZW_to_eulerXYZ(quaternion_xyzw):  # pylint: disable=invalid-name
  """Abstraction for converting from quaternion to a 3-parameter toation.

  This will help us easily switch which rotation parameterization we use.
  Quaternion should be in xyzw order for pybullet.

  Args:
    quaternion_xyzw: in xyzw order, tuple of 4 floats

  Returns:
    rotation: a 3-parameter rotation, in xyz order, tuple of 3 floats
  """
  q = quaternion_xyzw
  quaternion_wxyz = np.array([q[3], q[0], q[1], q[2]])
  euler_zxy = euler.quat2euler(quaternion_wxyz, axes='szxy')
  euler_xyz = (euler_zxy[1], euler_zxy[2], euler_zxy[0])
  return euler_xyz


def apply_transform(transform_to_from, points_from):
  r"""Transforms points (3D) into new frame.

  Using transform_to_from notation.

  Args:
    transform_to_from: numpy.ndarray of shape [B,4,4], SE3
    points_from: numpy.ndarray of shape [B,3,N]

  Returns:
    points_to: numpy.ndarray of shape [B,3,N]
  """
  num_points = points_from.shape[-1]

  # non-batched
  if len(transform_to_from.shape) == 2:
    ones = np.ones((1, num_points))

    # makes these each into homogenous vectors
    points_from = np.vstack((points_from, ones))  # [4,N]
    points_to = transform_to_from @ points_from  # [4,N]
    return points_to[0:3, :]  # [3,N]

  # batched
  else:
    assert len(transform_to_from.shape) == 3
    batch_size = transform_to_from.shape[0]
    zeros = np.ones((batch_size, 1, num_points))
    points_from = np.concatenate((points_from, zeros), axis=1)
    assert points_from.shape[1] == 4
    points_to = transform_to_from @ points_from
    return points_to[:, 0:3, :]


#-----------------------------------------------------------------------------
# IMAGE UTILS
#-----------------------------------------------------------------------------


def preprocess(img):
  """Pre-process input (subtract mean, divide by std)."""
  color_mean = 0.18877631
  depth_mean = 0.00509261
  color_std = 0.07276466
  depth_std = 0.00903967
  img[:, :, :3] = (img[:, :, :3] / 255 - color_mean) / color_std
  img[:, :, 3:] = (img[:, :, 3:] - depth_mean) / depth_std
  return img


def get_fused_heightmap(obs, configs, bounds, pix_size):
  """Reconstruct orthographic heightmaps with segmentation masks."""
  heightmaps, colormaps = reconstruct_heightmaps(
      obs['color'], obs['depth'], configs, bounds, pix_size)
  colormaps = np.float32(colormaps)
  heightmaps = np.float32(heightmaps)

  # Fuse maps from different views.
  valid = np.sum(colormaps, axis=3) > 0
  repeat = np.sum(valid, axis=0)
  repeat[repeat == 0] = 1
  cmap = np.sum(colormaps, axis=0) / repeat[Ellipsis, None]
  cmap = np.uint8(np.round(cmap))
  hmap = np.max(heightmaps, axis=0)  # Max to handle occlusions.
  return cmap, hmap


def get_image_transform(theta, trans, pivot=(0, 0)):
  """Compute composite 2D rigid transformation matrix."""
  # Get 2D rigid transformation matrix that rotates an image by theta (in
  # radians) around pivot (in pixels) and translates by trans vector (in
  # pixels)
  pivot_t_image = np.array([[1., 0., -pivot[0]], [0., 1., -pivot[1]],
                            [0., 0., 1.]])
  image_t_pivot = np.array([[1., 0., pivot[0]], [0., 1., pivot[1]],
                            [0., 0., 1.]])
  transform = np.array([[np.cos(theta), -np.sin(theta), trans[0]],
                        [np.sin(theta), np.cos(theta), trans[1]], [0., 0., 1.]])
  return np.dot(image_t_pivot, np.dot(transform, pivot_t_image))


def check_transform(image, pixel, transform):
  """Valid transform only if pixel locations are still in FoV after transform."""
  new_pixel = np.flip(
      np.int32(
          np.round(
              np.dot(transform,
                     np.float32([pixel[1], pixel[0],
                                 1.]).reshape(3, 1))))[:2].squeeze())
  valid = np.all(
      new_pixel >= 0
  ) and new_pixel[0] < image.shape[0] and new_pixel[1] < image.shape[1]
  return valid, new_pixel


def get_se3_from_image_transform(theta, trans, pivot, heightmap, bounds,
                                 pixel_size):
  """Calculate SE3 from image transform."""
  position_center = pix_to_xyz(
      np.flip(np.int32(np.round(pivot))),
      heightmap,
      bounds,
      pixel_size,
      skip_height=False)
  new_position_center = pix_to_xyz(
      np.flip(np.int32(np.round(pivot + trans))),
      heightmap,
      bounds,
      pixel_size,
      skip_height=True)
  # Don't look up the z height, it might get augmented out of frame
  new_position_center = (new_position_center[0], new_position_center[1],
                         position_center[2])

  delta_position = np.array(new_position_center) - np.array(position_center)

  t_world_center = np.eye(4)
  t_world_center[0:3, 3] = np.array(position_center)

  t_centernew_center = np.eye(4)
  euler_zxy = (-theta, 0, 0)
  t_centernew_center[0:3, 0:3] = euler.euler2mat(
      *euler_zxy, axes='szxy')[0:3, 0:3]

  t_centernew_center_tonly = np.eye(4)
  t_centernew_center_tonly[0:3, 3] = -delta_position
  t_centernew_center = t_centernew_center @ t_centernew_center_tonly

  t_world_centernew = t_world_center @ np.linalg.inv(t_centernew_center)
  return t_world_center, t_world_centernew


def get_random_image_transform_params(image_size):
  theta_sigma = 2 * np.pi / 6
  theta = np.random.normal(0, theta_sigma)

  trans_sigma = np.min(image_size) / 6
  trans = np.random.normal(0, trans_sigma, size=2)  # [x, y]
  pivot = (image_size[1] / 2, image_size[0] / 2)
  return theta, trans, pivot


def perturb(input_image, pixels, set_theta_zero=False):
  """Data augmentation on images."""
  image_size = input_image.shape[:2]

  # Compute random rigid transform.
  while True:
    theta, trans, pivot = get_random_image_transform_params(image_size)
    if set_theta_zero:
      theta = 0.
    transform = get_image_transform(theta, trans, pivot)
    transform_params = theta, trans, pivot

    # Ensure pixels remain in the image after transform.
    is_valid = True
    new_pixels = []
    new_rounded_pixels = []
    for pixel in pixels:
      pixel = np.float32([pixel[1], pixel[0], 1.]).reshape(3, 1)

      rounded_pixel = np.int32(np.round(transform @ pixel))[:2].squeeze()
      rounded_pixel = np.flip(rounded_pixel)

      pixel = (transform @ pixel)[:2].squeeze()
      pixel = np.flip(pixel)

      in_fov_rounded = rounded_pixel[0] < image_size[0] and rounded_pixel[
          1] < image_size[1]
      in_fov = pixel[0] < image_size[0] and pixel[1] < image_size[1]

      is_valid = is_valid and np.all(rounded_pixel >= 0) and np.all(
          pixel >= 0) and in_fov_rounded and in_fov

      new_pixels.append(pixel)
      new_rounded_pixels.append(rounded_pixel)
    if is_valid:
      break

  # Apply rigid transform to image and pixel labels.
  input_image = cv2.warpAffine(
      input_image,
      transform[:2, :], (image_size[1], image_size[0]),
      flags=cv2.INTER_NEAREST)
  return input_image, new_pixels, new_rounded_pixels, transform_params


#-----------------------------------------------------------------------------
# PLOT UTILS
#-----------------------------------------------------------------------------

# Plot colors (Tableau palette).
COLORS = {
    'blue': [078.0 / 255.0, 121.0 / 255.0, 167.0 / 255.0],
    'red': [255.0 / 255.0, 087.0 / 255.0, 089.0 / 255.0],
    'green': [089.0 / 255.0, 169.0 / 255.0, 079.0 / 255.0],
    'orange': [242.0 / 255.0, 142.0 / 255.0, 043.0 / 255.0],
    'yellow': [237.0 / 255.0, 201.0 / 255.0, 072.0 / 255.0],
    'purple': [176.0 / 255.0, 122.0 / 255.0, 161.0 / 255.0],
    'pink': [255.0 / 255.0, 157.0 / 255.0, 167.0 / 255.0],
    'cyan': [118.0 / 255.0, 183.0 / 255.0, 178.0 / 255.0],
    'brown': [156.0 / 255.0, 117.0 / 255.0, 095.0 / 255.0],
    'gray': [186.0 / 255.0, 176.0 / 255.0, 172.0 / 255.0]
}


def plot(fname,  # pylint: disable=dangerous-default-value
         title,
         ylabel,
         xlabel,
         data,
         xlim=[-np.inf, 0],
         xticks=None,
         ylim=[np.inf, -np.inf],
         show_std=True):
  """Plot frame data."""
  # Data is a dictionary that maps experiment names to tuples with 3
  # elements: x (size N array) and y (size N array) and y_std (size N array)

  # Get data limits.
  for name, (x, y, _) in data.items():
    del name
    y = np.array(y)
    xlim[0] = max(xlim[0], np.min(x))
    xlim[1] = max(xlim[1], np.max(x))
    ylim[0] = min(ylim[0], np.min(y))
    ylim[1] = max(ylim[1], np.max(y))

  # Draw background.
  plt.title(title, fontsize=14)
  plt.ylim(ylim)
  plt.ylabel(ylabel, fontsize=14)
  plt.yticks(fontsize=14)
  plt.xlim(xlim)
  plt.xlabel(xlabel, fontsize=14)
  plt.grid(True, linestyle='-', color=[0.8, 0.8, 0.8])
  ax = plt.gca()
  for axis in ['top', 'bottom', 'left', 'right']:
    ax.spines[axis].set_color('#000000')
  plt.rcParams.update({'font.size': 14})
  plt.rcParams['mathtext.default'] = 'regular'
  matplotlib.rcParams['pdf.fonttype'] = 42
  matplotlib.rcParams['ps.fonttype'] = 42

  # Draw data.
  color_iter = 0
  for name, (x, y, std) in data.items():
    del name
    x, y, std = np.float32(x), np.float32(y), np.float32(std)
    upper = np.clip(y + std, ylim[0], ylim[1])
    lower = np.clip(y - std, ylim[0], ylim[1])
    color = COLORS[list(COLORS.keys())[color_iter]]
    if show_std:
      plt.fill_between(x, upper, lower, color=color, linewidth=0, alpha=0.3)
    plt.plot(x, y, color=color, linewidth=2, marker='o', alpha=1.)
    color_iter += 1

  if xticks:
    plt.xticks(ticks=range(len(xticks)), labels=xticks, fontsize=14)
  else:
    plt.xticks(fontsize=14)
  plt.legend([name for name, _ in data.items()],
             loc='lower right', fontsize=14)
  plt.tight_layout()
  plt.savefig(fname)
  plt.clf()


#-----------------------------------------------------------------------------
# MESHCAT UTILS
#-----------------------------------------------------------------------------


def create_visualizer(clear=True):
  print('Waiting for meshcat server... have you started a server?')
  vis = meshcat.Visualizer(zmq_url='tcp://127.0.0.1:6000')
  if clear:
    vis.delete()
  return vis


def make_frame(vis, name, h, radius, o=1.0):
  """Add a red-green-blue triad to the Meschat visualizer.

  Args:
    vis (MeshCat Visualizer): the visualizer
    name (string): name for this frame (should be unique)
    h (float): height of frame visualization
    radius (float): radius of frame visualization
    o (float): opacity
  """
  vis[name]['x'].set_object(
      g.Cylinder(height=h, radius=radius),
      g.MeshLambertMaterial(color=0xff0000, reflectivity=0.8, opacity=o))
  rotate_x = mtf.rotation_matrix(np.pi / 2.0, [0, 0, 1])
  rotate_x[0, 3] = h / 2
  vis[name]['x'].set_transform(rotate_x)

  vis[name]['y'].set_object(
      g.Cylinder(height=h, radius=radius),
      g.MeshLambertMaterial(color=0x00ff00, reflectivity=0.8, opacity=o))
  rotate_y = mtf.rotation_matrix(np.pi / 2.0, [0, 1, 0])
  rotate_y[1, 3] = h / 2
  vis[name]['y'].set_transform(rotate_y)

  vis[name]['z'].set_object(
      g.Cylinder(height=h, radius=radius),
      g.MeshLambertMaterial(color=0x0000ff, reflectivity=0.8, opacity=o))
  rotate_z = mtf.rotation_matrix(np.pi / 2.0, [1, 0, 0])
  rotate_z[2, 3] = h / 2
  vis[name]['z'].set_transform(rotate_z)


def meshcat_visualize(vis, obs, act, info):
  """Visualize data using meshcat."""

  for key in sorted(info.keys()):

    pose = info[key]
    pick_transform = np.eye(4)
    pick_transform[0:3, 3] = pose[0]
    quaternion_wxyz = np.asarray(
        [pose[1][3], pose[1][0], pose[1][1], pose[1][2]])
    pick_transform[0:3, 0:3] = mtf.quaternion_matrix(quaternion_wxyz)[0:3, 0:3]
    label = 'obj_' + str(key)
    make_frame(vis, label, h=0.05, radius=0.0012, o=1.0)
    vis[label].set_transform(pick_transform)

  for cam_index in range(len(act['camera_config'])):

    verts = unproject_depth_vectorized(
        obs['depth'][cam_index], np.array([0, 1]),
        np.array(act['camera_config'][cam_index]['intrinsics']).reshape(3, 3),
        np.zeros(5))

    # switch from [N,3] to [3,N]
    verts = verts.T

    cam_transform = np.eye(4)
    cam_transform[0:3, 3] = act['camera_config'][cam_index]['position']
    quaternion_xyzw = act['camera_config'][cam_index]['rotation']
    quaternion_wxyz = np.asarray([
        quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1],
        quaternion_xyzw[2]
    ])
    cam_transform[0:3, 0:3] = mtf.quaternion_matrix(quaternion_wxyz)[0:3, 0:3]
    verts = apply_transform(cam_transform, verts)

    colors = obs['color'][cam_index].reshape(-1, 3).T / 255.0

    vis['pointclouds/' + str(cam_index)].set_object(
        g.PointCloud(position=verts, color=colors))

  def degrees_to_radians(degrees):
    """
    Converts degrees to radians
    @param: degrees
    """
    radians = (degrees / 360.0) * 2 * np.pi
    return radians


def V2T(V):
  """
  V2T converts 1x6 vector into 4x4 transformation matrix
  @param: V - 1x6 vector of form [x,y,z,rx,ry,rz] where x,y,z is the translation
  and rx,ry,rz is an angle-axis representation of the angle where the
  unit vector representing the axis has been multipled by the angle of
  rotation about it.
  @returns: T - a standard 4x4 transformation matrix
  """
  assert (V.shape == (1, 6))
  T = np.eye(4)
  T[0:3, 3] = V[0, 0:3]
  T[0:3, 0:3] = V2R(V[0, 3:6])
  return T


def V2R(V):
  """
  V2R converts a 1x3 angle-axis vector into a 3x3 rotation matrix
  @param: V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis
  representation of the angle where the unit vector representing the axis
  has been multipled by the angle of rotation about it.
  @returns: R - a standard 3x3 transformation matrix
  """
  V = V.transpose()
  s = np.linalg.norm(V)
  if s == 0:
    R = np.eye(3)
  else:
    V = V[:] / s
    V = V.reshape(3, 1)
    V = np.insert(V, 3, s)
    R = vrrotvec2mat(V)

  return R


def T2V(T):
  """
  T2V converts 4x4 transformation matrix into a 1x6 vector
  @param: T - a standard 4x4 transformation matrix
  @returns:V - 1x6 vector of form [x,y,z,rx,ry,rz] where x,y,z is the translation
  and rx,ry,rz is an angle-axis representation of the angle where the
  unit vector representing the axis has been multipled by the angle of
  rotation about it
  """
  assert (T.shape == (4, 4))
  V = np.zeros((1, 6))
  V[0, 0:3] = T[0:3, 3]
  V[0, 3:6] = R2V(T[0:3, 0:3])

  return V


def R2V(R):
  """
  R2V converts 3x3 rotation matrix into a 1x3 angle-axis vector
  @param: R - a standard 3x3 transformation matrix
  @returns: V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis
  representation of the angle where the unit vector representing the axis
  has been multipled by the angle of rotation about it
  """
  assert (R.shape == (3, 3))
  R = vrrotmat2vec(R)
  V = R[0, 0:3] * R[0, 2]

  return V


def vrrotvec2mat(ax_ang):
  """
  Create a rotation matrix corresponding to the rotation around a general
  axis by a specified angle.
  """
  if ax_ang.ndim == 1:
    if np.size(ax_ang) == 5:
      ax_ang = np.reshape(ax_ang, (5, 1))
      msz = 1
    elif np.size(ax_ang) == 4:
      ax_ang = np.reshape(np.hstack((ax_ang, np.array([1]))), (5, 1))
      msz = 1
    else:
      raise Exception('Wrong Input Type')
  elif ax_ang.ndim == 2:
    if np.shape(ax_ang)[0] == 5:
      msz = np.shape(ax_ang)[1]
    elif np.shape(ax_ang)[1] == 5:
      ax_ang = ax_ang.transpose()
      msz = np.shape(ax_ang)[1]
    else:
      raise Exception('Wrong Inpuqt Type')
  else:
    raise Exception('Wrong Input Type')

  direction = ax_ang[0:3, :]
  angle = ax_ang[3, :]

  d = np.array(direction, dtype=np.float64)
  d /= np.linalg.norm(d, axis=0)
  x = d[0, :]
  y = d[1, :]
  z = d[2, :]
  c = np.cos(angle)
  s = np.sin(angle)
  tc = 1 - c

  mt11 = tc * x * x + c
  mt12 = tc * x * y - s * z
  mt13 = tc * x * z + s * y

  mt21 = tc * x * y + s * z
  mt22 = tc * y * y + c
  mt23 = tc * y * z - s * x

  mt31 = tc * x * z - s * y
  mt32 = tc * y * z + s * x
  mt33 = tc * z * z + c

  mtx = np.column_stack((mt11, mt12, mt13, mt21, mt22, mt23, mt31, mt32, mt33))

  inds1 = np.where(ax_ang[4, :] == -1)
  mtx[inds1, :] = -mtx[inds1, :]

  if msz == 1:
    mtx = mtx.reshape(3, 3)
  else:
    mtx = mtx.reshape(msz, 3, 3)

  return mtx


def vrrotmat2vec(mat1, rot_type='proper'):
  """
  Create an axis-angle np.array from Rotation Matrix:
  ====================

  @param mat:  The nx3x3 rotation matrices to convert
  @type mat:   nx3x3 numpy array

  @param rot_type: 'improper' if there is a possibility of
                    having improper matrices in the input,
                    'proper' otherwise. 'proper' by default
  @type  rot_type: string ('proper' or 'improper')

  @return:    The 3D rotation axis and angle (ax_ang)
              5 entries:
                 First 3: axis
                 4: angle
                 5: 1 for proper and -1 for improper
  @rtype:     numpy 5xn array

  """
  mat = np.copy(mat1)
  if mat.ndim == 2:
    if np.shape(mat) == (3, 3):
      mat = np.copy(np.reshape(mat, (1, 3, 3)))
    else:
      raise Exception('Wrong Input Typef')
  elif mat.ndim == 3:
    if np.shape(mat)[1:] != (3, 3):
      raise Exception('Wrong Input Typez')
  else:
    raise Exception('Wrong Input Type')

  msz = np.shape(mat)[0]
  ax_ang = np.zeros((5, msz))

  epsilon = 1e-12
  if rot_type == 'proper':
    ax_ang[4, :] = np.ones(np.shape(ax_ang[4, :]))
  elif rot_type == 'improper':
    for i in range(msz):
      det1 = np.linalg.det(mat[i, :, :])
      if abs(det1 - 1) < epsilon:
        ax_ang[4, i] = 1
      elif abs(det1 + 1) < epsilon:
        ax_ang[4, i] = -1
        mat[i, :, :] = -mat[i, :, :]
      else:
        raise Exception('Matrix is not a rotation: |det| != 1')
  else:
    raise Exception('Wrong Input parameter for rot_type')

  mtrc = mat[:, 0, 0] + mat[:, 1, 1] + mat[:, 2, 2]

  ind1 = np.where(abs(mtrc - 3) <= epsilon)[0]
  ind1_sz = np.size(ind1)
  if np.size(ind1) > 0:
    ax_ang[:4, ind1] = np.tile(np.array([0, 1, 0, 0]), (ind1_sz, 1)).transpose()

  ind2 = np.where(abs(mtrc + 1) <= epsilon)[0]
  ind2_sz = np.size(ind2)
  if ind2_sz > 0:
    # phi = pi
    # This singularity requires elaborate sign ambiguity resolution

    # Compute axis of rotation, make sure all elements >= 0
    # real signs are obtained by flipping algorithm below
    diag_elems = np.concatenate((mat[ind2, 0, 0].reshape(ind2_sz, 1),
                                 mat[ind2, 1, 1].reshape(ind2_sz, 1),
                                 mat[ind2, 2, 2].reshape(ind2_sz, 1)), axis=1)
    axis = np.sqrt(np.maximum((diag_elems + 1) / 2, np.zeros((ind2_sz, 3))))
    # axis elements that are <= epsilon are set to zero
    axis = axis * ((axis > epsilon).astype(int))

    # Flipping
    #
    # The algorithm uses the elements above diagonal to determine the signs
    # of rotation axis coordinate in the singular case Phi = pi.
    # All valid combinations of 0, positive and negative values lead to
    # 3 different cases:
    # If (Sum(signs)) >= 0 ... leave all coordinates positive
    # If (Sum(signs)) == -1 and all values are non-zero
    #   ... flip the coordinate that is missing in the term that has + sign,
    #       e.g. if 2AyAz is positive, flip x
    # If (Sum(signs)) == -1 and 2 values are zero
    #   ... flip the coord next to the one with non-zero value
    #   ... ambiguous, we have chosen shift right

    # construct vector [M23 M13 M12] ~ [2AyAz 2AxAz 2AxAy]
    # (in the order to facilitate flipping):    ^
    #                                  [no_x  no_y  no_z ]

    m_upper = np.concatenate((mat[ind2, 1, 2].reshape(ind2_sz, 1),
                              mat[ind2, 0, 2].reshape(ind2_sz, 1),
                              mat[ind2, 0, 1].reshape(ind2_sz, 1)), axis=1)

    # elements with || smaller than epsilon are considered to be zero
    signs = np.sign(m_upper) * ((abs(m_upper) > epsilon).astype(int))

    sum_signs = np.sum(signs, axis=1)
    t1 = np.zeros(ind2_sz, )
    tind1 = np.where(sum_signs >= 0)[0]
    t1[tind1] = np.ones(np.shape(tind1))

    tind2 = np.where(np.all(np.vstack(((np.any(signs == 0, axis=1) == False), t1 == 0)), axis=0))[0]
    t1[tind2] = 2 * np.ones(np.shape(tind2))

    tind3 = np.where(t1 == 0)[0]
    flip = np.zeros((ind2_sz, 3))
    flip[tind1, :] = np.ones((np.shape(tind1)[0], 3))
    flip[tind2, :] = np.copy(-signs[tind2, :])

    t2 = np.copy(signs[tind3, :])

    shifted = np.column_stack((t2[:, 2], t2[:, 0], t2[:, 1]))
    flip[tind3, :] = np.copy(shifted + (shifted == 0).astype(int))

    axis = axis * flip
    ax_ang[:4, ind2] = np.vstack((axis.transpose(), np.pi * (np.ones((1, ind2_sz)))))

  ind3 = np.where(np.all(np.vstack((abs(mtrc + 1) > epsilon, abs(mtrc - 3) > epsilon)), axis=0))[0]
  ind3_sz = np.size(ind3)
  if ind3_sz > 0:
    phi = np.arccos((mtrc[ind3] - 1) / 2)
    den = 2 * np.sin(phi)
    a1 = (mat[ind3, 2, 1] - mat[ind3, 1, 2]) / den
    a2 = (mat[ind3, 0, 2] - mat[ind3, 2, 0]) / den
    a3 = (mat[ind3, 1, 0] - mat[ind3, 0, 1]) / den
    axis = np.column_stack((a1, a2, a3))
    ax_ang[:4, ind3] = np.vstack((axis.transpose(), phi.transpose()))

  return ax_ang

