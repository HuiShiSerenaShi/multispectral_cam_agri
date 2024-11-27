import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import numpy as np
from src.calibration.calibration_loader import CalibrationLoader
from src.reconstruction.point_cloud_reconstructor import PointCloudReconstructor

# Load calibration data
calibration_loader = CalibrationLoader("calib_dataset/saved_results.mat")
calibration_loader.load_calibration()

# Load depth and RGB images
depth_image = np.load("example_depth.npy")  
rgb_image = np.load("example_rgb.npy") 

# Generate point cloud
reconstructor = PointCloudReconstructor(calibration_loader)
point_cloud = reconstructor.generate_point_cloud(depth_image, rgb_image)

# Save point cloud
reconstructor.save_point_cloud(point_cloud, "results/reconstructed_point_cloud.ply")
