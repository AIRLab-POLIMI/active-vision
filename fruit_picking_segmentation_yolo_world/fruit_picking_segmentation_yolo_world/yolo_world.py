import rclpy
import rclpy.duration
from rclpy.node import Node
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image as SensorImage
from cv_bridge import CvBridge

import cv2
import torch
import numpy as np

from inference.models.yolo_world.yolo_world import YOLOWorld as YOLOWorld_model
from efficientvit.models.efficientvit.sam import EfficientViTSamPredictor
from efficientvit.sam_model_zoo import create_sam_model
import supervision as sv

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from queue import Queue
import threading

from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import KeyValue

from fruit_picking_interfaces.msg import ImageArray, Confidence



class YOLOWorldNode(Node):

    def __init__(self):
        super().__init__('yolo_world_node')
        
        # Create bridge between cv2 images and sensor_msgs/Image type
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("rgb_image_topic", "rgb_image"),
                ("depth_image_topic", "depth_image"),
                ("depth_image_camera_info_topic", "depth_image_camera_info"),
                ("yolo_world_rgb_image_topic", "/yolo_world/rgb_image"),
                ("yolo_world_rgb_images_array_topic", "/yolo_world/rgb_images_array"),
                ("yolo_world_depth_image_topic", "/yolo_world/depth_image"),
                ("yolo_world_depth_image_camera_info_topic", "/yolo_world/camera_info"),
                ("confidences_topic", "/yolo_world/confidences"),
                ("yolo_world_tf_topic", "/yolo_world/tf"),
                ("frame_id", "world"),
                ("publish_masks_array", True),
                ("publish_original_depth_image", True),
                ("publish_original_depth_image_camera_info", True),
                ("publish_original_tf", True),
                ("yolo_world_model_type", "yolo_world/l"),
                ("efficient_SAM_model_type", "l0"),
                ("segmentation_prompt", "tomato"),
                ("confidence_threshold", 0.001),
                ("nms_threshold", 0.2),
            ],
        )

        self._rgb_image_topic = self.get_parameter("rgb_image_topic").value
        self._depth_image_topic = self.get_parameter("depth_image_topic").value
        self._depth_image_camera_info_topic = self.get_parameter("depth_image_camera_info_topic").value
        self._yolo_world_rgb_image_topic = self.get_parameter("yolo_world_rgb_image_topic").value
        self._yolo_world_rgb_images_array_topic = self.get_parameter("yolo_world_rgb_images_array_topic").value
        self._yolo_world_depth_image_topic = self.get_parameter("yolo_world_depth_image_topic").value
        self._yolo_world_depth_image_camera_info_topic = self.get_parameter("yolo_world_depth_image_camera_info_topic").value
        self._confidences_topic = self.get_parameter("confidences_topic").value
        self._yolo_world_tf_topic = self.get_parameter("yolo_world_tf_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._publish_masks_array = self.get_parameter("publish_masks_array").value
        self._publish_original_depth_image = self.get_parameter("publish_original_depth_image").value
        self._publish_original_depth_image_camera_info = self.get_parameter("publish_original_depth_image_camera_info").value
        self._publish_original_tf = self.get_parameter("publish_original_tf").value
        self._yolo_world_model_type = self.get_parameter("yolo_world_model_type").value
        self._efficient_SAM_model_type = self.get_parameter("efficient_SAM_model_type").value
        self._yolo_world_segmentation_prompt = self.get_parameter("segmentation_prompt").value
        self._yolo_world_confidence_threshold = self.get_parameter("confidence_threshold").value
        self._yolo_world_nms_threshold = self.get_parameter("nms_threshold").value



    
        # Load YOLO World model
        self.get_logger().info(
            f"[INIT] Loading YOLO World model ({self._yolo_world_model_type}). This may take some time..."
        )
        self._yolo_world = YOLOWorld_model(model_id=self._yolo_world_model_type)
        self.get_logger().info("[INIT] YOLO World model loaded.")



        # Load Efficient SAM model and annotators
        self.get_logger().info(
            f"[INIT] Loading Efficient SAM model ({self._efficient_SAM_model_type}) and annotators. This may take some time..."
        )
        self._device = "cuda" if torch.cuda.is_available() else "cpu"
        self._sam = EfficientViTSamPredictor(
            create_sam_model(name=self._efficient_SAM_model_type, weight_url=f"../michelelagreca/Documents/robotics/models/efficient_SAM_{self._efficient_SAM_model_type}.pt").to(self._device).eval()
        )

        self._MASK_ANNOTATOR = sv.MaskAnnotator()

        self.get_logger().info("[INIT] Efficient SAM model and annotators loaded.")

        




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
        self.image_publisher = self.create_publisher(SensorImage, self._yolo_world_rgb_image_topic, 10)
        self.get_logger().info(f'[INIT] Merged masks publisher is ready.')

        if (self._publish_masks_array):
            self.image_array_publisher = self.create_publisher(ImageArray, self._yolo_world_rgb_images_array_topic, 10)
            self.get_logger().info(f'[INIT] Masks array publisher is ready.')

        # Define confidence publisher
        self.confidences_publisher = self.create_publisher(Confidence, self._confidences_topic, 10)

        # Define depth image publisher
        if (self._publish_original_depth_image):
            self.depth_image_publisher = self.create_publisher(SensorImage, self._yolo_world_depth_image_topic, 10)
            self.get_logger().info(f'[INIT] Original depth image publisher is ready.')


        # Define depth image camera info publisher
        if (self._publish_original_depth_image_camera_info):
            self.depth_image_camera_info_publisher = self.create_publisher(CameraInfo, self._yolo_world_depth_image_camera_info_topic, 10)
            self.get_logger().info(f'[INIT] Original depth image camera info publisher is ready.')

        # Define tf publisher
        if (self._publish_original_tf):
            self.tf_publisher = self.create_publisher(TransformStamped, self._yolo_world_tf_topic, 10)
            self.get_logger().info(f"[INIT] TF of the original data's publisher is ready.")



        
        self.get_logger().info(f'[INIT] YOLO World node is ready.')




    def clock_sub(self, msg):
        pass
             



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
        self.get_logger().debug('[Confidences-pub] Confidences published.')           
  
    


    def segmentation_wrapper(self, rgb_msg, depth_msg, depth_image_camera_info_msg):
        if (self.lock == True):
            try:
                # Wait for the transform to become available
                self.tf_buffer.can_transform(self._frame_id, depth_image_camera_info_msg.header.frame_id, depth_image_camera_info_msg.header.stamp, timeout=rclpy.duration.Duration(seconds=2.0))
                t = self.tf_buffer.lookup_transform(
                    self._frame_id,
                    depth_image_camera_info_msg.header.frame_id,
                    depth_image_camera_info_msg.header.stamp)
            except TransformException as ex:
                self.get_logger().warn(
                    f'Could not transform {self._frame_id} to {depth_image_camera_info_msg.header.frame_id}: {ex}')
                return
            # Put the synchronized messages and the transform into the queue
            self.get_logger().debug(f'[YOLOWorld] [segmentation_wrapper] Before the put the queue size is {self.segmentation_queue.qsize()}')
            self.segmentation_queue.put((rgb_msg, depth_msg, depth_image_camera_info_msg, t))
            self.get_logger().debug(f'[YOLOWorld] [segmentation_wrapper] After the put the queue size is {self.segmentation_queue.qsize()}')
            self.lock = False




    def segmentation_thread_func(self):
        while rclpy.ok():
            if (self.lock == False):
                # Get the valid data when the queue is filled with it
                self.get_logger().debug(f'[YOLOWorld] [segmentation_thread_func] Before the get the queue size is {self.segmentation_queue.qsize()}')
                rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg = self.segmentation_queue.get()
                self.get_logger().debug(f'[YOLOWorld] [segmentation_thread_func] After the get the queue size is {self.segmentation_queue.qsize()}')
                # Call the actual segmentation callback
                self.segment(rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg)
                self.lock = True




    def segment(self, rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg):

        # Get input data
        self.get_logger().info('------------------------------------------------')
        self.get_logger().info('[YOLOWorld] Original image received.')
        self.original_image = rgb_msg
        self.depth_image = depth_msg
        self.depth_image_camera_info = depth_image_camera_info_msg
        self.tf = tf_msg




        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image), cv2.COLOR_BGR2RGB)
        img_shape = img.shape



        # Get prompt
        text_prompt_query = self._yolo_world_segmentation_prompt




        # Segmentation
        self.get_logger().info(
            f"[YOLOWorld] Segmenting image of shape {img_shape} with text prompt prior: {text_prompt_query}..."
        )

        # Saving text prompt into an array
        categories = [category.strip() for category in text_prompt_query.split(",")]

        # Set the prompt to YOLO World
        self._yolo_world.set_classes(categories)


        # Inference with Yolo World
        start_detection = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"[YOLOWorld] Detection with confidence threshold {self._yolo_world_confidence_threshold} and nms confidence threshold {self._yolo_world_nms_threshold}."
        )
        self.get_logger().info(
            f"[YOLOWorld] Detection starting time: {start_detection}."
        )
        results = self._yolo_world.infer(img, confidence=self._yolo_world_confidence_threshold)
        detections = sv.Detections.from_inference(results).with_nms(
            class_agnostic=True, threshold=self._yolo_world_nms_threshold
        )


        # Segmentation with Efficient SAM
        start_segmentation = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"[YOLOWorld] Segmentation starting time: {start_segmentation}."
        )
        self._sam.set_image(img, image_format="RGB")
        masks = []
        for xyxy in detections.xyxy:
            mask, _, _ = self._sam.predict(box=xyxy, multimask_output=False)
            masks.append(mask.squeeze())

        self.get_logger().info(
            f"[YOLOWorld] Detection (YOLO World) completed in {round((start_segmentation - start_detection)/1.e9, 5)}s."
        )
        self.get_logger().info(
            f"[YOLOWorld] Segmentation (Efficient SAM) completed in {round((self.get_clock().now().nanoseconds - start_segmentation)/1.e9, 5)}s."
        )
        self.get_logger().info(
            f"[YOLOWorld] Total inference completed in {round((self.get_clock().now().nanoseconds - start_detection)/1.e9, 5)}s."
        )

        detections.mask = np.array(masks)


        # Obtain confidences
        confidences = detections.confidence



        # Start publishing phase

        # Set the initial time of this phase
        start_pub = self.get_clock().now().nanoseconds
        start_pub_stamp = self.get_clock().now().to_msg()

    
        # Check number of masks found
        self.get_logger().info(
            f"[YOLOWorld] Masks found: {len(masks)}."
        )


        # Declare a new ImageArray object
        if (self._publish_masks_array):
            mask_images_array = ImageArray()
            mask_images_array.header.frame_id = self.original_image.header.frame_id
            mask_images_array.header.stamp = start_pub_stamp



        # Case when the model did not segment any mask, thus the result is a null image
        if len(masks) <= 0:

            # Publish confidences as empty lists
            self.get_logger().debug(f'[Confidences-pub] Publishing empty confidences...')
            self.publish_confidences_segmentation(torch.empty_like(torch.tensor([])), [], 'none', self.original_image.header.frame_id, start_pub_stamp)
            
            # Create empty image
            empty_image = np.ones((img_shape[0], img_shape[1], 3), dtype=np.uint8) * 255
            empty_image = cv2.cvtColor(empty_image, cv2.COLOR_BGR2RGB)
            empty_image = self.bridge.cv2_to_imgmsg(empty_image, encoding="rgb8")
            empty_image.header.frame_id = self.original_image.header.frame_id
            empty_image.header.stamp = start_pub_stamp


            # Publish empty image
            self.image_publisher.publish(empty_image)
            self.get_logger().debug('[Image-pub] Empty image published.')


            # Case when the array of masks need to be published
            if (self._publish_masks_array):
            
                # Append the white image inside the images array
                mask_images_array.images.append(empty_image)
                
                 # Publish empty image array
                self.image_array_publisher.publish(mask_images_array)
                self.get_logger().debug('[ImageArray-pub] Empty image array published.')
            
            # Case when the original depth image needs to be published
            if (self._publish_original_depth_image):
                self.depth_image.header.stamp = start_pub_stamp
                self.depth_image_publisher.publish(self.depth_image)
                self.get_logger().debug('[DepthImage-pub] Depth image published.')

            # Case when the original depth image camera info needs to be published
            if (self._publish_original_depth_image_camera_info):
                self.depth_image_camera_info.header.stamp = start_pub_stamp
                self.depth_image_camera_info_publisher.publish(self.depth_image_camera_info)
                self.get_logger().debug('[DepthImageCameraInfo-pub] Depth image camera info published.')
            
            # Case when the original tf needs to be published
            if (self._publish_original_tf):
                self.tf.header.stamp = start_pub_stamp
                self.tf_publisher.publish(self.tf)
                self.get_logger().debug('[TF-pub] TF message republished.')
                

                

        # Case when the model segmented some masks
        else:
            masks_names = [f"mask_{i}" for i in range(1, len(masks) + 1)]

            # Publish confidences
            self.get_logger().debug(f'[Confidences-pub] Publishing confidences...')
            self.publish_confidences_segmentation(confidences, masks_names, text_prompt_query, self.original_image.header.frame_id, start_pub_stamp)

            # Merged masks publication  
            merged_masks_images = np.ones((img.shape[0], img.shape[1], 3), dtype=np.uint8) * 255
            merged_masks_images = cv2.cvtColor(merged_masks_images, cv2.COLOR_BGR2RGB)
            merged_masks_images = self._MASK_ANNOTATOR.annotate(merged_masks_images, detections)
            merged_masks_images = self.bridge.cv2_to_imgmsg(merged_masks_images, encoding="rgb8")
            merged_masks_images.header.frame_id = self.original_image.header.frame_id
            merged_masks_images.header.stamp = start_pub_stamp

            # Define image publisher and publish
            self.image_publisher.publish(merged_masks_images)
            self.get_logger().debug('[Image-pub] Merged masks image published.')


            # Image array publication
            if (self._publish_masks_array):
                for i in range(len(detections.xyxy)):
                    single_detection = sv.Detections.empty()
                    single_detection.xyxy = np.array([detections.xyxy[i]])
                    single_detection.mask = np.array([masks[i]])
                    single_detection.confidence = np.array([detections.confidence[i]])
                    single_detection.class_id = np.array([detections.class_id[i]])
                    single_detection.tracker_id = None
                    single_detection.data = {'class_name': np.array([detections.data['class_name'][i]])}
                    merged_masks_images = np.ones((img.shape[0], img.shape[1], 3), dtype=np.uint8) * 255
                    merged_masks_images = cv2.cvtColor(merged_masks_images, cv2.COLOR_BGR2RGB)
                    merged_masks_images = self._MASK_ANNOTATOR.annotate(merged_masks_images, single_detection)
                    mask_image = self.bridge.cv2_to_imgmsg(merged_masks_images, encoding="rgb8")
                    mask_image.header.frame_id = self.original_image.header.frame_id
                    mask_image.header.stamp = start_pub_stamp

                    # Add each image in the ImageArray object
                    mask_images_array.images.append(mask_image)
                self.image_array_publisher.publish(mask_images_array)
                self.get_logger().debug('[ImageArray-pub] Image array published.')
                

            # Case when the original depth image needs to be published
            if (self._publish_original_depth_image):
                self.depth_image.header.stamp = start_pub_stamp
                self.depth_image_publisher.publish(self.depth_image)
                self.get_logger().debug('[DepthImage-pub] Depth image published.')

            # Case when the original depth image camera info needs to be published
            if (self._publish_original_depth_image_camera_info):
                self.depth_image_camera_info.header.stamp = start_pub_stamp
                self.depth_image_camera_info_publisher.publish(self.depth_image_camera_info)
                self.get_logger().debug('[DepthImageCameraInfo-pub] Depth image camera info published.')
            
            # Case when the original tf needs to be published
            if (self._publish_original_tf):
                self.tf.header.stamp = start_pub_stamp
                self.tf_publisher.publish(self.tf)
                self.get_logger().debug('[TF-pub] TF message republished.')


        self.get_logger().info(
            f"[Pub] From the received moment, data published in {round((self.get_clock().now().nanoseconds - start_pub)/1.e9, 5)}s."
        )



def main(args=None):

    # Init of the node
    rclpy.init(args=args)
    node = YOLOWorldNode()

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