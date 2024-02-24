import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory

import os
import cv2
from lang_sam import LangSAM
from PIL import Image
import torch
import numpy as np

from fruit_picking_segmentation_lang_sam.utils import merge_masks_images, convert_masks_to_images, rgba_to_rgb_with_white_background, export_merged_masks_images


class LANGSAMPubSub(Node):


    # Init method
    def __init__(self):
        super().__init__('lang_sam_pub_sub')
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("model_type", "vit_h"),
                ("input_image_topic", "/virtual_camera_link/rgbd_camera/image_raw"),
                ("output_image_topic", "/fruit_picking/segmentation/lang_sam/image"),
            ],
        )

        self._model_type = self.get_parameter("model_type").value
        self._input_image_topic = self.get_parameter("input_image_topic").value
        self._output_image_topic = self.get_parameter("output_image_topic").value




        # Load LANG SAM model
        self.get_logger().info(
            f"Loading LANG SAM model '{self._model_type}'. This may take some time..."
        )

        self._lang_sam = LangSAM(
            self._model_type,
        )

        self.get_logger().info("LANG SAM model loaded.")




        # Define publisher and subscriber
        self.publisher = self.create_publisher(SensorImage, self._output_image_topic, 10)
        self.subscription = self.create_subscription(
            SensorImage, self._input_image_topic, self.segment, 10)

     


        self.get_logger().info(f'Pub-Sub client is ready.')



    def publish_segmentation(self, image_msg):

        # If merged masks image coming from the server is not node, it is published
        if image_msg is not None:
            self.publisher.publish(image_msg)
            self.get_logger().info('Merged masks published.')
        else:
            self.get_logger().info('Nothing published.')




    def segment(self, msg):

        # Get input image from input topic
        self.get_logger().info('Original image received.')
        self.original_image = msg


        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image), cv2.COLOR_BGR2RGB)
        img_query = Image.fromarray(img)


        # Get prompt
        text_prompt_query = "plant"


        # Segmentation
        self.get_logger().info(
            f"Segmenting image of shape {img.shape} with text prompt prior: {text_prompt_query}"
        )
        start = self.get_clock().now().nanoseconds

        masks, boxes, phrases, logits = self._lang_sam.predict(img_query, text_prompt_query)

        self.get_logger().info(
            f"Segmentation completed in {round((self.get_clock().now().nanoseconds - start)/1.e9, 2)}s."
        )


        # Check number of masks found
        self.get_logger().info(
            f"Masks found: {masks.size(0)}."
        )


        if masks.size(0) <= 0:
            merged_masks_images = None

        else:
            # Prepare merged masks images to be published
            # Convert bool masks tensor to cv2 images
            masks_images = convert_masks_to_images(masks)
            merged_masks_images = merge_masks_images(masks_images)
            merged_masks_images = rgba_to_rgb_with_white_background(merged_masks_images)
            merged_masks_images = np.uint8(merged_masks_images * 255 * 255) # in order to visualize the color image for RViz and also for the exported image

            # In order to visualize in Rviz, the image need to be processed more
            merged_masks_images = cv2.convertScaleAbs(merged_masks_images)
            merged_masks_images = self.bridge.cv2_to_imgmsg(merged_masks_images)
            
            # Convert boxes tensor in std_msgs/Float64MultiArray
            boxes = boxes.squeeze().cpu().numpy().astype(float).ravel().tolist()
            msg_boxes = Float64MultiArray()
            msg_boxes.data = boxes

            # Convert confidences tensor in list fo float rounded at the 3rd digit
            confidences = [round(logit.item(), 3) for logit in logits]


        self.publish_segmentation(merged_masks_images)


    


def main(args=None):
    rclpy.init(args=args)
    node = LANGSAMPubSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()