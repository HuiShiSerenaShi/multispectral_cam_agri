import numpy as np
import open3d as o3d


class ColorProjector:
    """
    Class for projecting colors from multiple cameras onto a 3D point cloud.
    """

    def __init__(self, calibration_loader, preprocessor=None):
        """
        Initialize the ColorProjector with a CalibrationLoader and optional preprocessor.
        :param calibration_loader: CalibrationLoader object for accessing camera parameters.
        :param preprocessor: Optional preprocessor object for handling image preprocessing.
        """
        self.calibration_loader = calibration_loader
        self.preprocessor = preprocessor  # Object for image preprocessing

    def project_colors(self, point_cloud, image, cam_name):
        """
        Project colors from a specified camera onto the point cloud.
        :param point_cloud: Open3D PointCloud object.
        :param image: Camera image as a NumPy array.
        :param cam_name: Name of the camera.
        :return: PointCloud with projected colors.
        """
        # Step 1: Preprocess the image (if a preprocessor is provided)
        if self.preprocessor:
            image = self.preprocessor.process(image, cam_name)

        # Step 2: Get intrinsic and extrinsic parameters
        intrinsics = self.calibration_loader.get_intrinsics(cam_name)
        extrinsics = self.calibration_loader.get_extrinsics("realsense_rgb", cam_name)

        fx, fy = intrinsics["focal_length"]
        cx, cy = intrinsics["principal_point"]
        R = np.array(extrinsics["rotation"])
        t = np.array(extrinsics["translation"])

        # Step 3: Transform points to the camera frame
        points = np.asarray(point_cloud.points)
        colors = np.asarray(point_cloud.colors)
        points_cam = (R @ points.T + t[:, None]).T
        points_cam = points_cam[points_cam[:, 2] > 0]  # Remove points behind the camera

        # Step 4: Project points onto the image plane
        x = (points_cam[:, 0] * fx / points_cam[:, 2]) + cx
        y = (points_cam[:, 1] * fy / points_cam[:, 2]) + cy
        x = np.round(x).astype(int)
        y = np.round(y).astype(int)

        # Step 5: Filter points outside the image bounds
        valid_mask = (x >= 0) & (x < image.shape[1]) & (y >= 0) & (y < image.shape[0])
        x = x[valid_mask]
        y = y[valid_mask]
        points_cam = points_cam[valid_mask]

        # Step 6: Map colors from the image to the point cloud
        mapped_colors = image[y, x] / 255.0  # Normalize to [0, 1]
        colors[valid_mask] = mapped_colors

        # Update point cloud colors
        point_cloud.colors = o3d.utility.Vector3dVector(colors)
        return point_cloud
