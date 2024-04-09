#!/usr/bin/env python

from argparse import ArgumentParser
import cv2
import apriltag
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

def inverse_matrix(pose_matrix):
    rotation_matrix = pose_matrix[:3, :3]
    translation_vector = pose_matrix[:3, 3]

    inv_rotation_matrix = np.linalg.inv(rotation_matrix)
    inv_translation_vector = -np.dot(inv_rotation_matrix, translation_vector)

    inverse_pose_matrix = np.zeros_like(pose_matrix)
    inverse_pose_matrix[:3, :3] = inv_rotation_matrix
    inverse_pose_matrix[:3, 3] = inv_translation_vector
    inverse_pose_matrix[3, 3] = 1

    return inverse_pose_matrix

def calculate_rotation_angles(rotation_matrix):
    rx = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    ry = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    rz = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return rx, ry, rz

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')
        self.publisher_distance = self.create_publisher(Vector3, 'distance_variation', 10)
        self.publisher_angles = self.create_publisher(Vector3, 'angle_variation', 10)

        # Initialize AprilTag detector
        parser = ArgumentParser(description='Detect AprilTags from video stream.')
        apriltag.add_arguments(parser)
        options = parser.parse_args()
        self.detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())
    
    def run_detection(self, frame):
        result, overlay = apriltag.detect_tags(frame, self.detector, camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                                tag_size=0.0762, vizualization=3, verbose=3, annotation=True)
    
        if result:
            pose_info = result[1]
            inverse_homography = inverse_matrix(pose_info)
            translation = inverse_homography[:3, 3]  # Camera's position relative to AprilTag
            rotation_angles = calculate_rotation_angles(inverse_homography[:3, :3])  # Camera's orientation relative to AprilTag

            # Calculate distance variation (positional variation)
            distance_variation = np.linalg.norm(translation)

            # Publish distance variation
            distance_msg = Vector3()
            distance_msg.x = distance_variation
            self.publisher_distance.publish(distance_msg)

            # Publish angle variation
            angles_msg = Vector3()
            angles_msg.x, angles_msg.y, angles_msg.z = rotation_angles
            self.publisher_angles.publish(angles_msg)

            # Display the calculated information
            self.get_logger().info("Distance Variation: %f" % distance_variation)
            self.get_logger().info("Angle Variation: X:%f, Y:%f, Z:%f" % rotation_angles)


def main(args=None):
    rclpy.init(args=args)
    april_tag_detector = AprilTagDetector()

    # Set up video capture
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    try:
        while rclpy.ok() and video.isOpened():
            success, frame = video.read()
            if not success:
                break

            frame = cv2.resize(frame, (160, 120))

            april_tag_detector.run_detection(frame)

            if cv2.waitKey(1) & 0xFF == ord(' '):
                break

    finally:
        video.release()
        cv2.destroyAllWindows()

    april_tag_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
