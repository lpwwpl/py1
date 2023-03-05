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

"""Camera configs."""

import numpy as np
import pybullet as p


class RealSenseD435():
  """Default configuration with 3 RealSense RGB-D cameras."""

  # Mimic RealSense D435 RGB-D camera parameters.
  image_size = (480, 640)
  intrinsics = (6.12692566e+02, 0, 3.23764923e+02, 0, 6.12443115e+02, 2.33248459e+02, 0, 0, 1)

  # Set default camera poses.
  front_position = (1., 0, 0.75)
  front_rotation = (np.pi / 4, np.pi, -np.pi / 2)
  front_rotation = p.getQuaternionFromEuler(front_rotation)

  # Default camera configs.
  CONFIG = {
      'image_size': image_size,
      'intrinsics': intrinsics,
      'position': front_position,
      'rotation': front_rotation,
      'zrange': (0.01, 10.),
      'noise': False
  }


class Oracle():
  """Top-down noiseless image used only by the oracle demonstrator."""

  # Near-orthographic projection.
  image_size = (480,640)
  intrinsics = (6.12692566e+02, 0, 3.23764923e+02, 0, 6.12443115e+02, 2.33248459e+02, 0, 0, 1)
  position = (0.0, 0, 0.35)
  # np.pi, -np.pi / 2
  rotation = p.getQuaternionFromEuler((0, np.pi, -np.pi / 2))
  # 6.12692566e+02
  # 0.00000000e+00
  # 3.23764923e+02
  # 0.00000000e+00
  # 6.12443115e+02
  # 2.33248459e+02
  # 0.00000000e+00
  # 0.00000000e+00
  # 1.00000000e+00

  # Camera config.
  CONFIG = [{
      'image_size': image_size,
      'intrinsics': intrinsics,
      'position': position,
      'rotation': rotation,
      'zrange': (0.01, 10.),
      'noise': False
  },
  {
      'image_size': image_size,
      'intrinsics': intrinsics,
      'position': position,
      'rotation': rotation,
      'zrange': (0.01, 10.),
      'noise': False
  }]
