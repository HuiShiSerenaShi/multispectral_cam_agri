import numpy as np
import open3d as o3d


class PointCloudReconstructor:
    """
    Class to reconstruct 3D point clouds using depth and RGB images.
    """

    def __init__(self, calibration_loader):
        """
        Initialize the PointCloudReconstructor with a CalibrationLoader.
        :param calibration_loader: CalibrationLoader object for accessing camera parameters.
        """
        self.calibration_loader = calibration_loader

    def generate_point_cloud(self, depth_image, rgb_image, cam_name="realsense_rgb"):
        """
        Generate a 3D point cloud from depth and RGB images.
        :param depth_image: Depth image as a NumPy array.
        :param rgb_image: RGB image as a NumPy array.
        :param cam_name: Name of the camera (default: realsense_rgb).
        :return: Open3D PointCloud object.
        """
        # Get intrinsic parameters for the camera
        intrinsics = self.calibration_loader.get_intrinsics(cam_name)
        fx, fy = intrinsics["focal_length"]
        cx, cy = intrinsics["principal_point"]

        # Create 3D point cloud
        h, w = depth_image.shape
        x, y = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_image / 1000.0  # Convert depth from mm to meters
        x3d = (x - cx) * z / fx
        y3d = (y - cy) * z / fy

        # Stack 3D points
        points = np.dstack((x3d, y3d, z)).reshape(-1, 3)

        # Remove invalid points
        valid_mask = (depth_image > 0)
        points = points[valid_mask.ravel()]
        colors = rgb_image.reshape(-1, 3)[valid_mask.ravel()] / 255.0

        # Create Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors)

        return point_cloud

    def save_point_cloud(self, point_cloud, output_path):
        """
        Save the generated point cloud to a file.
        :param point_cloud: Open3D PointCloud object.
        :param output_path: Path to save the point cloud file.
        """
        o3d.io.write_point_cloud(output_path, point_cloud)
        print(f"Point cloud saved to {output_path}")
