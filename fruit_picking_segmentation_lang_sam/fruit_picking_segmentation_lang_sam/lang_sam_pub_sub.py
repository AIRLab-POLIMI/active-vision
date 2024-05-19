import rclpy
import rclpy.duration
from rclpy.node import Node
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image as SensorImage
from cv_bridge import CvBridge

import cv2
from lang_sam import LangSAM
from PIL import Image
import torch
import numpy as np

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from queue import Queue
import threading

from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import KeyValue

from fruit_picking_segmentation_lang_sam.utils import merge_masks_images, convert_masks_to_images, rgba_to_rgb_with_white_background

from fruit_picking_interfaces.msg import ImageArray, Confidence



class LANGSAMPubSub(Node):

    def __init__(self):
        super().__init__('lang_sam_pub_sub')
        
        # Create bridge between cv2 images and sensor_msgs/Image type
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("sam_model_type", "vit_h"),
                ("rgb_image_topic", "rgb_image"),
                ("depth_image_topic", "depth_image"),
                ("depth_image_camera_info_topic", "depth_image_camera_info"),
                ("lang_sam_rgb_image_topic", "/lang_sam/rgb_image"),
                ("lang_sam_rgb_images_array_topic", "/lang_sam/rgb_images_array"),
                ("lang_sam_depth_image_topic", "/lang_sam/depth_image"),
                ("lang_sam_depth_image_camera_info_topic", "/lang_sam/camera_info"),
                ("confidences_topic", "/lang_sam/confidences"),
                ("lang_sam_tf_topic", "/lang_sam/tf"),
                ("frame_id", "world"),
                ("base_frame_id", "igus_rebel_base_link"), 
                ("publish_masks_array", True),
                ("publish_original_depth_image", True),
                ("publish_original_depth_image_camera_info", True),
                ("publish_original_tf", True),
                ("segmentation_prompt", "tomato"),
            ],
        )

        self._sam_model_type = self.get_parameter("sam_model_type").value
        self._rgb_image_topic = self.get_parameter("rgb_image_topic").value
        self._depth_image_topic = self.get_parameter("depth_image_topic").value
        self._depth_image_camera_info_topic = self.get_parameter("depth_image_camera_info_topic").value
        self._lang_sam_rgb_image_topic = self.get_parameter("lang_sam_rgb_image_topic").value
        self._lang_sam_rgb_images_array_topic = self.get_parameter("lang_sam_rgb_images_array_topic").value
        self._lang_sam_depth_image_topic = self.get_parameter("lang_sam_depth_image_topic").value
        self._lang_sam_depth_image_camera_info_topic = self.get_parameter("lang_sam_depth_image_camera_info_topic").value
        self._confidences_topic = self.get_parameter("confidences_topic").value
        self._lang_sam_tf_topic = self.get_parameter("lang_sam_tf_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._base_frame_id = self.get_parameter("base_frame_id").value
        self._publish_masks_array = self.get_parameter("publish_masks_array").value
        self._publish_original_depth_image = self.get_parameter("publish_original_depth_image").value
        self._publish_original_depth_image_camera_info = self.get_parameter("publish_original_depth_image_camera_info").value
        self._publish_original_tf = self.get_parameter("publish_original_tf").value
        self._lang_sam_segmentation_prompt = self.get_parameter("segmentation_prompt").value



    
        # Load LANG SAM model
        self.get_logger().info(
            f"[INIT] Loading LANG SAM model '{self._sam_model_type}'. This may take some time..."
        )

        self._lang_sam = LangSAM(
            self._sam_model_type,
        )

        self.get_logger().info("[INIT] LANG SAM model loaded.")




        # Define clock callback group for multi-threading and initialize clock subscriber
        clock_subscriber_group = MutuallyExclusiveCallbackGroup()
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_sub, 10, callback_group=clock_subscriber_group)




        # Define RGB image, depth image, depth image camera info subscribers
        self.rgb_image_sub = message_filters.Subscriber(self, SensorImage, self._rgb_image_topic)
        self.depth_image_sub = message_filters.Subscriber(self, SensorImage, self._depth_image_topic)
        self.depth_image_camera_info_sub = message_filters.Subscriber(self, CameraInfo, self._depth_image_camera_info_topic)



        # Define a tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)



        # Initialize the queue as an intermediary between the synchronizer and the real callback  and a bool to handle the access to it
        self.segmentation_queue = Queue()
        self.lock = True



        # Initialize the synchronizer registering a wrapper callback that save the data into the queue
        self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub, self.depth_image_sub, self.depth_image_camera_info_sub], 10)
        self.ts.registerCallback(self.segmentation_wrapper)



        # Start the segmentation thread
        self.segmentation_thread = threading.Thread(target=self.segmentation_thread_func)
        self.segmentation_thread.start()



        # Define image and image array publishers
        self.image_publisher = self.create_publisher(SensorImage, self._lang_sam_rgb_image_topic, 10)
        self.get_logger().info(f'[INIT] Merged masks publisher is ready.')

        if (self._publish_masks_array):
            self.image_array_publisher = self.create_publisher(ImageArray, self._lang_sam_rgb_images_array_topic, 10)
            self.get_logger().info(f'[INIT] Masks array publisher is ready.')

        # Define confidence publisher
        self.confidences_publisher = self.create_publisher(Confidence, self._confidences_topic, 10)

        # Define depth image publisher
        if (self._publish_original_depth_image):
            self.depth_image_publisher = self.create_publisher(SensorImage, self._lang_sam_depth_image_topic, 10)
            self.get_logger().info(f'[INIT] Original depth image publisher is ready.')


        # Define depth image camera info publisher
        if (self._publish_original_depth_image_camera_info):
            self.depth_image_camera_info_publisher = self.create_publisher(CameraInfo, self._lang_sam_depth_image_camera_info_topic, 10)
            self.get_logger().info(f'[INIT] Original depth image camera info publisher is ready.')

        # Define tf publisher
        if (self._publish_original_tf):
            self.tf_publisher = self.create_publisher(TransformStamped, self._lang_sam_tf_topic, 10)
            self.get_logger().info(f"[INIT] TF of the original data's publisher is ready.")



        
        self.get_logger().info(f'[INIT] Pub-Sub client is ready.')






    def clock_sub(self, msg):
        pass




    def publish_image_segmentation(self, image_msg):
        self.image_publisher.publish(image_msg)
        self.get_logger().info('[Image-pub] Image published.')




    def publish_image_array_segmentation(self, image_array_msg):
        self.image_array_publisher.publish(image_array_msg)
        self.get_logger().info('[ImageArray-pub] Image array published.')




    def publish_confidences_segmentation(self, confidences, masks_names, semantic_class, frame_id, stamp):
        # Convert confidences tensor in list of float rounded at the 3rd digit and 
        confidences = [round(logit.item(), 6) for logit in confidences]

        confidences_dict = dict(zip(masks_names, confidences))
        msg_confidences = Confidence()
        msg_confidences.data = [KeyValue(key=k, value=str(v)) for k, v in confidences_dict.items()] 
        msg_confidences.header.frame_id = frame_id
        msg_confidences.header.stamp = stamp
        msg_confidences.semantic_class = semantic_class

        self.confidences_publisher.publish(msg_confidences)
        self.get_logger().info('[Confidences-pub] Confidences published.')   




    def publish_original_depth_image(self, depth_msg):
        self.depth_image_publisher.publish(depth_msg)
        self.get_logger().info('[DepthImage-pub] Depth image published.')


    

    def publish_original_depth_image_camera_info(self, depth_image_camera_info_msg):
        self.depth_image_camera_info_publisher.publish(depth_image_camera_info_msg)
        self.get_logger().info('[DepthImageCameraInfo-pub] Depth image camera info published.')
    



    def segmentation_wrapper(self, rgb_msg, depth_msg, depth_image_camera_info_msg):
        if (self.lock == True):
            try:
                t = self.tf_buffer.lookup_transform(
                    self._frame_id,
                    depth_image_camera_info_msg.header.frame_id,
                    depth_image_camera_info_msg.header.stamp)
            except TransformException as ex:
                self.get_logger().warn(
                    f'Could not transform {self._frame_id} to {rgb_msg.header.frame_id}: {ex}')
                return
            # Put the synchronized messages and the transform into the queue
            self.get_logger().debug(f'[LANG-SAM] [segmentation_wrapper] Before the put the queue size is {self.segmentation_queue.qsize()}')
            self.segmentation_queue.put((rgb_msg, depth_msg, depth_image_camera_info_msg, t))
            self.get_logger().debug(f'[LANG-SAM] [segmentation_wrapper] After the put the queue size is {self.segmentation_queue.qsize()}')
            self.lock = False




    def segmentation_thread_func(self):
        while rclpy.ok():
            if (self.lock == False):
                # Get the valid data when the queue is filled with it
                self.get_logger().debug(f'[LANG-SAM] [segmentation_thread_func] Before the get the queue size is {self.segmentation_queue.qsize()}')
                rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg = self.segmentation_queue.get()
                self.get_logger().debug(f'[LANG-SAM] [segmentation_thread_func] After the get the queue size is {self.segmentation_queue.qsize()}')
                # Call the actual segmentation callback
                self.segment(rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg)
                self.lock = True




    def segment(self, rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg):

        # Get input data
        self.get_logger().info('------------------------------------------------')
        self.get_logger().info('[LANG-SAM] Original image received.')
        self.original_image = rgb_msg
        self.depth_image = depth_msg
        self.depth_image_camera_info = depth_image_camera_info_msg
        self.tf = tf_msg




        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image), cv2.COLOR_BGR2RGB)
        img_query = Image.fromarray(img) # from OpenCv to PIL image
        img_shape = img.shape



        # Get prompt
        text_prompt_query = self._lang_sam_segmentation_prompt



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
            f"[LANG-SAM] Segmentation completed in {round((end_seg - start_seg)/1.e9, 5)}s."
        )




        # Start publishing phase

        # Set the initial time of this phase
        start_pub = self.get_clock().now().nanoseconds
        start_pub_stamp = self.get_clock().now().to_msg()

    
        # Check number of masks found
        self.get_logger().info(
            f"[LANG-SAM] Masks found: {masks.size(0)}."
        )


        # Declare a new ImageArray object
        if (self._publish_masks_array):
            mask_images_array = ImageArray()
            mask_images_array.header.frame_id = self.original_image.header.frame_id
            mask_images_array.header.stamp = start_pub_stamp



        # Case when the model did not segment any mask, thus the result is a null image
        if masks.size(0) <= 0:

            # Publish confidences as empty lists
            self.get_logger().info(f'[Confidences-pub] Publishing empty confidences...')
            self.publish_confidences_segmentation(torch.empty_like(torch.tensor([])), [], 'none', self.original_image.header.frame_id, start_pub_stamp)
            
            # Create empty image
            empty_image = np.ones((img_shape[0], img_shape[1], 3), dtype=np.uint8) * 255
            empty_image = cv2.cvtColor(empty_image, cv2.COLOR_BGR2RGB)
            empty_image = self.bridge.cv2_to_imgmsg(empty_image, encoding="rgb8")
            empty_image.header.frame_id = self.original_image.header.frame_id
            empty_image.header.stamp = start_pub_stamp


            # Publish empty image
            self.get_logger().info(f'[Image-pub] Publishing empty image...')
            self.publish_image_segmentation(empty_image)


            # Case when the array of masks need to be published
            if (self._publish_masks_array):
            
                # Append the white image inside the images array
                mask_images_array.images.append(empty_image)
                
                 # Publish empty image array
                self.get_logger().info(f'[ImageArray-pub] Publishing empty images array...')
                self.publish_image_array_segmentation(mask_images_array)
            
            # Case when the original depth image needs to be published
            if (self._publish_original_depth_image):
                self.depth_image.header.stamp = start_pub_stamp
                self.get_logger().info(f'[DepthImage-pub] Publishing original depth image...')
                self.publish_original_depth_image(self.depth_image)

            # Case when the original depth image camera info needs to be published
            if (self._publish_original_depth_image_camera_info):
                self.depth_image_camera_info.header.stamp = start_pub_stamp
                self.get_logger().info(f'[DepthImageCameraInfo-pub] Publishing original depth image camera info...')
                self.publish_original_depth_image_camera_info(self.depth_image_camera_info)
            
            # Case when the original tf needs to be published
            if (self._publish_original_tf):
                self.tf.header.stamp = start_pub_stamp
                self.tf_publisher.publish(self.tf)
                self.get_logger().info('[TF-pub] TF message republished.')
                

                

        # Case when the model segmented some masks
        else:
            masks_names = [f"mask_{i}" for i in range(1, masks.size(0) + 1)]

            # Prepare merged masks images to be published
            # Convert bool masks tensor to cv2 images
            masks_images = convert_masks_to_images(masks)

            # Publish confidences
            self.get_logger().info(f'[Confidences-pub] Publishing confidences...')
            self.publish_confidences_segmentation(confidences, masks_names, text_prompt_query, self.original_image.header.frame_id, start_pub_stamp)

            # Merged masks publication  
            merged_masks_images = merge_masks_images(masks_images)
            merged_masks_images = rgba_to_rgb_with_white_background(merged_masks_images)
            # merged_masks_images = cv2.cvtColor(merged_masks_images, cv2.COLOR_BGR2RGB)
            merged_masks_images = np.uint8(merged_masks_images * 255 * 255) # in order to visualize the color image for RViz and also for the exported image

            # In order to visualize in Rviz, the image need to be processed more
            merged_masks_images = cv2.convertScaleAbs(merged_masks_images)
            merged_masks_images = self.bridge.cv2_to_imgmsg(merged_masks_images, encoding="rgb8")
            merged_masks_images.header.frame_id = self.original_image.header.frame_id
            merged_masks_images.header.stamp = start_pub_stamp

            # Define image publisher and publish
            self.get_logger().info(f'[Image-pub] Publishing merged masks image...')
            self.publish_image_segmentation(merged_masks_images)


            # Image array publication
            if (self._publish_masks_array):
                for mask_image in masks_images:
                    mask_image = rgba_to_rgb_with_white_background(mask_image)
                    # mask_image = cv2.cvtColor(mask_image, cv2.COLOR_BGR2RGB)
                    mask_image = np.uint8(mask_image * 255 * 255) # in order to visualize the color image for RViz and also for the exported image

                    # In order to visualize in Rviz, the image need to be processed more
                    mask_image = cv2.convertScaleAbs(mask_image)
                    mask_image = self.bridge.cv2_to_imgmsg(mask_image, encoding="rgb8")
                    mask_image.header.frame_id = self.original_image.header.frame_id
                    mask_image.header.stamp = start_pub_stamp

                    # Add each image in the ImageArray object
                    mask_images_array.images.append(mask_image)
                self.get_logger().info(f'[ImageArray-pub] Publishing images array...')
                self.publish_image_array_segmentation(mask_images_array)

            # Case when the original depth image needs to be published
            if (self._publish_original_depth_image):
                self.depth_image.header.stamp = start_pub_stamp
                self.get_logger().info(f'[DepthImage-pub] Publishing original depth image...')
                self.publish_original_depth_image(self.depth_image)

            # Case when the original depth image camera info needs to be published
            if (self._publish_original_depth_image_camera_info):
                self.depth_image_camera_info.header.stamp = start_pub_stamp
                self.get_logger().info(f'[DepthImageCameraInfo-pub] Publishing original depth image camera info...')
                self.publish_original_depth_image_camera_info(self.depth_image_camera_info)
            
            # Case when the original tf needs to be published
            if (self._publish_original_tf):
                self.tf.header.stamp = start_pub_stamp
                self.tf_publisher.publish(self.tf)
                self.get_logger().info('[TF-pub] TF message republished.')


        self.get_logger().info(
            f"[Pub] From the received moment, data published in {round((self.get_clock().now().nanoseconds - start_pub)/1.e9, 5)}s."
        )



def main(args=None):

    # Init of the node
    rclpy.init(args=args)
    node = LANGSAMPubSub()

    # Creation of the multi thread executor for the node
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin the executor
    executor.spin()

    # Terminate
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()