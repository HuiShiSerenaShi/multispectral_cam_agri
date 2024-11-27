import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.calibration.calibration_loader import CalibrationLoader

# Initialize the loader
calibration_file = "calib_dataset/saved_results.mat"
loader = CalibrationLoader(calibration_file)

# Load the calibration data
loader.load_calibration()

# Get all camera pairs
camera_pairs = loader.get_camera_pairs()
print("Camera pairs:", camera_pairs)

# Get extrinsics between RealSense RGB and Thermal cameras
extrinsics = loader.get_extrinsics("realsense_rgb", "thermal")
print("Extrinsics:", extrinsics)

# Get intrinsics for the RealSense RGB camera
intrinsics = loader.get_intrinsics("realsense_rgb")
print("Intrinsics:", intrinsics)
