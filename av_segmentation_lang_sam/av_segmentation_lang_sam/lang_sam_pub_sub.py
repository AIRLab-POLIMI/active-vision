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

from av_segmentation_lang_sam.utils import merge_masks_images, convert_masks_to_images, rgba_to_rgb_with_white_background

from av_interfaces.msg import ImageArray, Confidence



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




        """
        In order to create a segmented pointcloud combining the segmented RGB image (both with SAM or color filtering) 
        and the depth image coming from the camera, the two data need to have the same timestamp.

        Infact, right before publish the segmented RGB image into the topic, the header timestamp has to be set to 
        the current ROS time. 

        - This is not a problem for the color filter processing, because it is a fast approach and the timestamp 
          before the publication can be set also equal to the timestamp of the original image before the processing, 
          even though **the current time would be the best theoric setting**.
        - Using the current ROS time is mandatory for the segmented RGB image with SAM, because between the start of 
          the inference and its end there could be some processing time. Thus, using the original image timestamp could 
          lead in using an old timestamp, and in such a way the final processed RGB image would have a timestamp different 
          to the current depth image.
          - The case becomes as the color filter segmentation if the inference is done using a GPU, and as a result the 
            inference time becomes negligible.

        This is doable if ROS time is used (`use_sim_time` parameter set to `False`)

        When the `use_sim_time` parameter is set to `True`, everything change. Infact, in this case, getting the 
        current ROS time in the node is not as simple as before, because:

        - with `use_sim_time` set to `False`, the time is the ROS time, and it is internal and handled by the ROS system.
          It can be accessed anytime by each node even inside the callbacks without subscribing to any topic 
          (eg without subscribing to `/clock` topic). This time is always updated.
    
        - with `use_sim_time` set to `True`, the time is the simulation time (eg the Gazebo Ignition time). 
          ROS obtain the time subscribing to the `/clock` topic.
    
          Thus, a node in order to get the ROS time has to subscribe to this topic. This subscription is done 
          internally in each node. The problem is that by default nodes are mono-threaded. So when a node is busy 
          doing a callback, until it finish this callback the simulated clock cannot be updated (the internal callback of 
          the `/clock` topic cannot be served in parallel of the current callback). This problem is visible when the 
          `lang_sam_pub_sub.py` file is executed, and the `LANGSAMPubSub` node is created.
    
            - This node is by default mono-threaded
            - When this node executed the callback segment, the unique thread is becoming busy for some second to do inference.
            - Inside the callback, if the node want to access the ROS time (using `get_clock().now()` for example), 
              it will access the ROS time published before the start of the current callcback, since the node can not 
              subscibe to the /clock topic because is already in a callback.

        The solution is multi-threading. In this way segment() will not have overlap with itself, and it will execute in 
        parallel with clock_callback()
        """

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



    """
    To solve the problem of multithreading between clock and segment callbacks, and the discrepancy between the resulting 
    segmented rgb image and the depth, camera and tf messages that needs to have the timestamp of the original image, 
    a possible solution is a subscription to 3 different topics in the `lang_sam_pub_sub.py` node. 
    The topics are `/depth_image`, `/rgb_image`, `/camera_info`. The idea is that, in this way, after the processing, 
    the mask are published, together with the original depth image and camera info topics, since these two messages 
    will be the exact data related to the processed RGB image. Infact this will avoid the fact that the depth image 
    and camera info taken will be new data not related to the processed image.

    Note that this problem is only when a decentralized apporach (suing topics) is used, and in the project this approach
    is used to perform octomap creation.

    - Instead of a subscriber to the RGB image topic, a synchronizer needs to be used, since the callback need to be 
      called after the reception of 3 messages from different topics.
    - In order to let the callback be executed on a different thread to deal with the previous case of subscription to 
      the `/clock` topic, a new approach has been used:
        - The previous setup used two mutually exclusive callback groups, one for the clock callback and another 
            for the segmentation callback. This ensured that each callback could be executed in separate threads, 
            allowing them to run concurrently without blocking each other. The segmentation callback was directly 
            subscribed to a topic that published RGB images, triggering the callback whenever a new image was published.
        - The current implementation explicitly defines only one mutually exclusive callback group for the clock subscriber. 
            This suggests that the primary intention is to isolate the clock callback execution, ensuring it runs 
            independently of other operations.
        - Instead of using a callback group for the segmentation operation, the current implementation employs a 
            custom threading solution.
            - It uses `message_filters` to synchronize messages from multiple topics.
            - The synchronization mechanism waits for three messages (presumably RGB image, depth image, and camera 
                info, based on the context) that arrive within a specified time frame. This ensures that the data being 
                processed is temporally coherent, which is often critical for tasks like image segmentation where data from 
                multiple sensors must align.
            - Once the synchronized messages are received, the wrapper function `segmentation_wrapper()` is triggered. 
                This function's role is to package these messages together (also with the related tf) and insert them into 
                a queue. The queue serves as a thread-safe mechanism for transferring data between threads, ensuring that 
                the data is not corrupted and that access is managed correctly across threads.
            - A separate thread runs continuously in the background executing the function `segmentation_thread_func()`, 
                monitoring the queue for new data. When new data arrives (i.e., the queue is not empty), this thread 
                retrieves the data and executes the semantic segmentation callback with it.
            - The semantic segmentation callback, which is executed by the dedicated thread, processes the synchronized 
                messages. Since this processing is done in a separate thread, it does not block or interfere with other 
                operations, such as receiving new messages or handling other callbacks (e.g., the clock callback).
        - To avoid the fact that the synchronizer put each message received into the queue, and instead put a single 
            message that will be immediately processed by `segmentation_thread_func`, a flag is used:
            - This flag is a boolean variable that is set to true initially
            - `segmentation_wrapper` is entered only if the lock is true, and after putting the current synchronized 
                data into the queue, it sets the lock to false
            - `segmentation_thread_func` is entered only if the lock is false. After the segment callback is executed, 
                the lock is set to true
        - This approach ensures that the `segmentation_wrapper` will only add items to the queue when the 
            `segmentation_thread_func` has processed the current item and explicitly signaled that it's safe to do so. 
            This approach does not guarantee atomicity and the same performances of the mutex. Still, for simplicity it 
            works (in the semantic segmentation approach).

    Another important element is getting the transforms of the system related to the specific input data 
    (RGB, depth and camera info). For this, the `lang_sam_pub_sub.py` node is also listening for the tf of the system. 
    Whenever a tuple of RGB, depth and camera info data is obtained by the callback and processed, in that moment also 
    the tf of the system is saved and published when the segmented image is available, since it is then used in the 
    octomap node to transform in a correct way the data.
    """
             



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

        masks, _, _, confidences = self._lang_sam.predict(img_query, text_prompt_query)

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
            masks_names = [f"mask_{i}" for i in range(1, masks.size(0) + 1)]

            # Prepare merged masks images to be published
            # Convert bool masks tensor to cv2 images
            masks_images = convert_masks_to_images(masks)

            # Publish confidences
            self.get_logger().debug(f'[Confidences-pub] Publishing confidences...')
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
            self.image_publisher.publish(merged_masks_images)
            self.get_logger().debug('[Image-pub] Merged masks image published.')


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