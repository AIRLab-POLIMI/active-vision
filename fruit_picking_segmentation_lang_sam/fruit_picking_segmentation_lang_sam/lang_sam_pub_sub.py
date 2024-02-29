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


class LANGSAMPubSub(Node):

    # This variable is used to save the number of masks found after each segmentation, in order to reset the topics 
    # of these masks the next segmentation
    masks_counter = 0



    def __init__(self):
        super().__init__('lang_sam_pub_sub')
        
        # Create bridge between cv2 images and sensor_msgs/Image type
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("model_type", "vit_h"),
                ("multiple_output_topics", True),
                ("input_image_topic", "/virtual_camera_link/rgbd_camera/image_raw"),
                ("output_image_topic", "/fruit_picking/segmentation/lang_sam/image"),
                ("output_boxes_topic", "/fruit_picking/segmentation/lang_sam/boxes"),
                ("output_confidences_topic", "/fruit_picking/segmentation/lang_sam/confidences"),
                ("segmentation_prompt", "tomato"),
            ],
        )

        self._model_type = self.get_parameter("model_type").value
        self._multiple_output_topics = self.get_parameter("multiple_output_topics").value
        self._input_image_topic = self.get_parameter("input_image_topic").value
        self._output_image_topic = self.get_parameter("output_image_topic").value
        self._output_boxes_topic = self.get_parameter("output_boxes_topic").value
        self._output_confidences_topic = self.get_parameter("output_confidences_topic").value
        self._segmentation_prompt = self.get_parameter("segmentation_prompt").value




    
        # Load LANG SAM model
        self.get_logger().info(
            f"[INIT] Loading LANG SAM model '{self._model_type}'. This may take some time..."
        )

        self._lang_sam = LangSAM(
            self._model_type,
        )

        self.get_logger().info("[INIT] LANG SAM model loaded.")


        # Define image subscriber
        self.subscription = self.create_subscription(
            SensorImage, self._input_image_topic, self.segment, 10)


        # Define boxes and confidences publishers
        self.boxes_publisher = self.create_publisher(String, self._output_boxes_topic, 10)
        self.confidences_publisher = self.create_publisher(String, self._output_confidences_topic, 10)



        self.get_logger().info(f'[INIT] Pub-Sub client is ready.')




    def publish_segmentation(self, image_msg):
        self.publisher.publish(image_msg)
        self.get_logger().info('[Image-pub] Image published.')



    def publish_boxes_segmentation(self, boxes, masks_names):

        # Convert boxes tensor in list of lists and publish as a json string
        boxes = boxes.tolist()
        boxes_dict = dict(zip(masks_names, boxes))
        boxes_json_str = json.dumps(boxes_dict)  # Serialize dictionary to JSON string
        msg_boxes = String()
        msg_boxes.data = boxes_json_str # to deserialize: confidences_dict = json.loads(confidences_json_str)

        self.boxes_publisher.publish(msg_boxes)
        self.get_logger().info('[Boxes-pub] Boxes published.')




    def publish_confidences_segmentation(self, confidences, masks_names):

        # Convert confidences tensor in list fo float rounded at the 3rd digit and 
        confidences = [round(logit.item(), 3) for logit in confidences]
        # Publish confidences list as a json string
        confidences_dict = dict(zip(masks_names, confidences))
        confidences_json_str = json.dumps(confidences_dict)  # Serialize dictionary to JSON string
        msg_confidences = String()
        msg_confidences.data = confidences_json_str # to deserialize: confidences_dict = json.loads(confidences_json_str)

        self.confidences_publisher.publish(msg_confidences)
        self.get_logger().info('[Confidences-pub] Confidences published.')   
    



    def segment(self, msg):

        # Get input image from input topic, and size
        self.get_logger().info('------------------------------------------------')
        self.get_logger().info('[LANG-SAM] Original image received.')
        self.original_image = msg



        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image), cv2.COLOR_BGR2RGB)
        img_query = Image.fromarray(img)
        img_shape = img.shape



        # Get prompt
        text_prompt_query = self._segmentation_prompt



        # Segmentation
        start_seg = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f"[LANG-SAM] Segmenting image of shape {img_shape} with text prompt prior: {text_prompt_query}..."
        )
        self.get_logger().info(
            f"[LANG-SAM] Inference starting time: {start_seg}."
        )

        masks, boxes, phrases, confidences = self._lang_sam.predict(img_query, text_prompt_query)

        end_seg = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f"[LANG-SAM] Inference ended at time: {end_seg}."
        )

        self.get_logger().info(
            f"[LANG-SAM] Segmentation completed in {round((end_seg - start_seg)/1.e9, 5)}s."
        )




        # Start publishing phase

        # Set the initial time of this phase
        start_pub = self.get_clock().now().nanoseconds

        # Reset all the masks topics to white image
        if self._multiple_output_topics:
            for i in range(0, LANGSAMPubSub.masks_counter):
                
                self.get_logger().info(f'[Image-pub] Resetting mask_{i + 1}...')
                white_image = np.ones((img_shape[0], img_shape[1], 3), dtype=np.uint8) * 255
                white_image = cv2.cvtColor(white_image, cv2.COLOR_BGR2RGB)
                white_image = self.bridge.cv2_to_imgmsg(white_image, encoding="rgb8")
                white_image.header.frame_id = self.original_image.header.frame_id
                white_image.header.stamp = self.get_clock().now().to_msg()
                
                reset_output_topic = f"{self._output_image_topic}/mask_{i + 1}"
                self.publisher = self.create_publisher(SensorImage, reset_output_topic, 10)
                self.publish_segmentation(white_image)



        # Check number of masks found
        self.get_logger().info(
            f"[LANG-SAM] Masks found: {masks.size(0)}."
        )



        # Case when the model did not segment any mask, thus the result is a null image
        if masks.size(0) <= 0:
            white_image = np.ones((img_shape[0], img_shape[1], 3), dtype=np.uint8) * 255
            white_image = cv2.cvtColor(white_image, cv2.COLOR_BGR2RGB)
            white_image = self.bridge.cv2_to_imgmsg(white_image, encoding="rgb8")
            white_image.header.frame_id = self.original_image.header.frame_id
            white_image.header.stamp = self.get_clock().now().to_msg()

            # Define publisher and publish
            self.get_logger().info(f'[Image-pub] Publishing empty image...')
            self.publisher = self.create_publisher(SensorImage, self._output_image_topic, 10)
            self.publish_segmentation(white_image)

            # Publish boxes and confidences as empty lists
            self.get_logger().info(f'[Boxes-pub] Publishing empty boxes...')
            self.publish_boxes_segmentation(torch.empty_like(torch.tensor([])), [])
            self.get_logger().info(f'[Confidences-pub] Publishing empty confidences...')
            self.publish_confidences_segmentation(torch.empty_like(torch.tensor([])), [])
            
            

        # Case when the model segmented some masks
        else:

            masks_names = [f"mask_{i}" for i in range(1, 5)]

            # Prepare merged masks images to be published
            # Convert bool masks tensor to cv2 images
            masks_images = convert_masks_to_images(masks)


            # Publish boxes and confidences
            self.get_logger().info(f'[Boxes-pub] Publishing boxes...')
            self.publish_boxes_segmentation(boxes, masks_names)
            self.get_logger().info(f'[Confidences-pub] Publishing confidences...')
            self.publish_confidences_segmentation(confidences, masks_names)




            # Case when publishing on multiple topics each mask is required
            if self._multiple_output_topics:

                for i, mask_image in enumerate(masks_images):
                    mask_image = rgba_to_rgb_with_white_background(mask_image)
                    # mask_image = cv2.cvtColor(mask_image, cv2.COLOR_BGR2RGB)
                    mask_image = np.uint8(mask_image * 255 * 255) # in order to visualize the color image for RViz and also for the exported image

                    # In order to visualize in Rviz, the image need to be processed more
                    mask_image = cv2.convertScaleAbs(mask_image)
                    mask_image = self.bridge.cv2_to_imgmsg(mask_image, encoding="rgb8")
                    mask_image.header.frame_id = self.original_image.header.frame_id
                    mask_image.header.stamp = self.get_clock().now().to_msg()

                    # Define publisher and publish
                    output_topic = f"{self._output_image_topic}/mask_{i + 1}"
                    self.publisher = self.create_publisher(SensorImage, output_topic, 10)
                    self.get_logger().info(f'[Image-pub] Publishing mask_{i + 1}...')
                    self.publish_segmentation(mask_image)


            # The merged masks images are always published
                    
            merged_masks_images = merge_masks_images(masks_images)
            merged_masks_images = rgba_to_rgb_with_white_background(merged_masks_images)
            # merged_masks_images = cv2.cvtColor(merged_masks_images, cv2.COLOR_BGR2RGB)
            merged_masks_images = np.uint8(merged_masks_images * 255 * 255) # in order to visualize the color image for RViz and also for the exported image

            # In order to visualize in Rviz, the image need to be processed more
            merged_masks_images = cv2.convertScaleAbs(merged_masks_images)
            merged_masks_images = self.bridge.cv2_to_imgmsg(merged_masks_images, encoding="rgb8")
            merged_masks_images.header.frame_id = self.original_image.header.frame_id
            merged_masks_images.header.stamp = self.get_clock().now().to_msg() 


            # Define publisher and publish
            self.publisher = self.create_publisher(SensorImage, self._output_image_topic, 10)
            self.get_logger().info(f'[Image-pub] Publishing merged masks images...')
            self.publish_segmentation(merged_masks_images)



        self.get_logger().info(
            f"[Pub] From the received moment, data published in {round((self.get_clock().now().nanoseconds - start_pub)/1.e9, 5)}s."
        )

        # Update the mask counter
        LANGSAMPubSub.masks_counter = masks.size(0)


def main(args=None):
    rclpy.init(args=args)
    node = LANGSAMPubSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()