import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import json

import cv2
from lang_sam import LangSAM
from PIL import Image
import torch
import numpy as np

from fruit_picking_segmentation_lang_sam.utils import merge_masks_images, convert_masks_to_images, rgba_to_rgb_with_white_background



class ColorFilter(Node):



    def __init__(self):
        super().__init__('color_filter')
        
        # Create bridge between cv2 images and sensor_msgs/Image type
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_image_topic", "/virtual_camera_link/rgbd_camera/image_raw"),
                ("output_image_topic", "/fruit_picking/segmentation/lang_sam/image"),
                ("colors", "red"),
            ],
        )

        self._input_image_topic = self.get_parameter("input_image_topic").value
        self._output_image_topic = self.get_parameter("output_image_topic").value
        self._colors = self.get_parameter("colors").value       


        self.get_logger().info(
            f"[INIT] Color fitering masks creation initializated."
        )
        # Define image subscriber
        self.subscription = self.create_subscription(
            SensorImage, self._input_image_topic, self.color_filtering, 10)



        self.get_logger().info(f'[INIT] Pub-Sub client is ready.')




    def publish_segmentation(self, image_msg):
        self.publisher.publish(image_msg)
        self.get_logger().info('[Image-pub] Image published.')
        



    def color_filtering(self, msg):
        
        # Get input image from input topic, and size
        self.get_logger().info('------------------------------------------------')
        self.original_image = msg



        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image), cv2.COLOR_BGR2RGB)
        self.get_logger().info('[COLOR-FILT] Original image received.')
        self.get_logger().info(f'[COLOR-FILT] Filtering {self._colors} zones...')

        start_filt = self.get_clock().now().nanoseconds

        # Convert to hsv
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)



        # Set filtering based on the desired color
        if self._colors == "red":
            # Red mask with red array regarding the hsv format
            lower_red = np.array([1, 0, 0])
            upper_red = np.array([10, 255, 255])
            color_mask = cv2.inRange(hsv, lower_red, upper_red)

        if self._colors == "green":
            # Green mask with red array regarding the hsv format
            lower_green = np.array([25, 0, 0])
            upper_green = np.array([45, 255, 255])
            color_mask = cv2.inRange(hsv, lower_green, upper_green)

        else: # case when both color red and green are desired to filter
            # Red mask with red array regarding the hsv format
            lower_red = np.array([1, 0, 0])
            upper_red = np.array([10, 255, 255])
            red_color_mask = cv2.inRange(hsv, lower_red, upper_red)
            # Green mask with red array regarding the hsv format
            lower_green = np.array([25, 0, 0])
            upper_green = np.array([45, 255, 255])
            green_color_mask = cv2.inRange(hsv, lower_green, upper_green)
            ## final mask and masked
            color_mask = cv2.bitwise_or(red_color_mask, green_color_mask)


        # Apply the mask to the image to keep only the desired color regions. The background is black
        filtered_img = cv2.bitwise_and(img, img, mask=color_mask)

        # Create a mask for black pixels
        black_mask = np.all(filtered_img == [0, 0, 0], axis=-1)

        # Replace black pixels with white
        filtered_img[black_mask] = [255, 255, 255]


        # In order to visualize in Rviz, the image need to be processed more
        filtered_img = np.uint8(filtered_img) # in order to visualize the color image for RViz and also for the exported image
        filtered_img = cv2.convertScaleAbs(filtered_img)
        filtered_img = self.bridge.cv2_to_imgmsg(filtered_img)
        self.get_logger().info(f'[COLOR-FILT] Image filtered.')


        # Define publisher and publish
        self.get_logger().info(f'[Image-pub] Publishing filtered image...')
        self.publisher = self.create_publisher(SensorImage, self._output_image_topic, 10)
        self.publish_segmentation(filtered_img)
        self.get_logger().info(
            f"[COLOR-FILT] Image filtered and published in {round((self.get_clock().now().nanoseconds - start_filt)/1.e9, 5)}s."
        )




def main(args=None):
    rclpy.init(args=args)
    node = ColorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()