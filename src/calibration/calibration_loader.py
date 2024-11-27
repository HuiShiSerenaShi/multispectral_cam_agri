import os
import scipy.io


class CalibrationLoader:
    """
    A class to load and manage camera calibration results from MATLAB-generated .mat files.
    """

    def __init__(self, calibration_file):
        """
        Initialize the CalibrationLoader with the path to the .mat calibration file.
        :param calibration_file: Path to the MATLAB calibration file.
        """
        self.calibration_file = calibration_file
        self.calibration_data = None

    def load_calibration(self):
        """
        Load the calibration data from the .mat file.
        """
        if not os.path.exists(self.calibration_file):
            raise FileNotFoundError(f"Calibration file not found: {self.calibration_file}")
        
        # Load MATLAB .mat file
        self.calibration_data = scipy.io.loadmat(self.calibration_file)
        print(f"Calibration data loaded successfully from {self.calibration_file}")

    def get_camera_pairs(self):
        """
        Get all camera pairs available in the calibration data.
        :return: List of camera pairs (e.g., [('realsense_rgb', 'thermal'), ('realsense_rgb', 'rgb'), ...])
        """
        if not self.calibration_data:
            raise ValueError("Calibration data is not loaded. Call `load_calibration()` first.")

        return list(self.calibration_data['calibrationResults'].dtype.names)

    def get_extrinsics(self, cam1, cam2):
        """
        Get extrinsic parameters between two cameras.
        :param cam1: Name of the first camera.
        :param cam2: Name of the second camera.
        :return: Extrinsic parameters (rotation matrix, translation vector).
        """
        if not self.calibration_data:
            raise ValueError("Calibration data is not loaded. Call `load_calibration()` first.")

        pair_key = f"{cam1}_{cam2}"
        if pair_key not in self.calibration_data['calibrationResults'].dtype.names:
            raise KeyError(f"Camera pair {cam1} <-> {cam2} not found in calibration results.")

        extrinsics = self.calibration_data['calibrationResults'][pair_key][0, 0]
        return {
            "rotation": extrinsics['cameraParams']['RotationOfCamera2'],
            "translation": extrinsics['cameraParams']['TranslationOfCamera2'],
        }

    def get_intrinsics(self, cam_name):
        """
        Get intrinsic parameters for a specific camera.
        :param cam_name: Name of the camera.
        :return: Intrinsic parameters (focal length, principal point, distortion coefficients).
        """
        if not self.calibration_data:
            raise ValueError("Calibration data is not loaded. Call `load_calibration()` first.")

        if cam_name not in self.calibration_data['imgData'].dtype.names:
            raise KeyError(f"Camera {cam_name} not found in calibration results.")

        intrinsics = self.calibration_data['imgData'][cam_name][0, 0]['cameraParams']
        return {
            "focal_length": intrinsics['FocalLength'],
            "principal_point": intrinsics['PrincipalPoint'],
            "distortion_coeffs": intrinsics['RadialDistortion'],
        }
