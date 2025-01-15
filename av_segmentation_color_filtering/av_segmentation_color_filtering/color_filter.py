import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as SensorImage
from cv_bridge import CvBridge
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock

import cv2
import numpy as np

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from queue import Queue
import threading


class ColorFilter(Node):


    def __init__(self):
        super().__init__('color_filter')
        
        # Create bridge between cv2 images and sensor_msgs/Image type
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("rgb_image_topic", "/rgb_image"),
                ("depth_image_topic", "depth_image"),
                ("depth_image_camera_info_topic", "depth_image_camera_info"),
                ("color_filter_rgb_image_topic", "/color_filter/rgb_image"),
                ("color_filter_depth_image_topic", "/color_filter/depth_image"),
                ("color_filter_depth_image_camera_info_topic", "/color_filter/camera_info"),
                ("color_filter_tf_topic", "/color_filter/tf"),
                ("frame_id", "world"),
                ("publish_original_depth_image", True),
                ("publish_original_depth_image_camera_info", True),
                ("publish_original_tf", True),
                ("colors", "red"),
            ],
        )


        self._rgb_image_topic = self.get_parameter("rgb_image_topic").value
        self._depth_image_topic = self.get_parameter("depth_image_topic").value
        self._depth_image_camera_info_topic = self.get_parameter("depth_image_camera_info_topic").value
        self._color_filter_rgb_image_topic = self.get_parameter("color_filter_rgb_image_topic").value
        self._color_filter_depth_image_topic = self.get_parameter("color_filter_depth_image_topic").value
        self._color_filter_depth_image_camera_info_topic = self.get_parameter("color_filter_depth_image_camera_info_topic").value
        self._color_filter_tf_topic = self.get_parameter("color_filter_tf_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._publish_original_depth_image = self.get_parameter("publish_original_depth_image").value
        self._publish_original_depth_image_camera_info = self.get_parameter("publish_original_depth_image_camera_info").value
        self._publish_original_tf = self.get_parameter("publish_original_tf").value
        self._colors = self.get_parameter("colors").value  




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



        # Initialize the queue as an intermediary between the synchronizer and the real callback, and a bool to handle the access to it
        self.queue = Queue()
        self.lock = True



        # Initialize the synchronizer registering a wrapper callback that save the data into the queue
        self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub, self.depth_image_sub, self.depth_image_camera_info_sub], 10)
        self.ts.registerCallback(self.color_filtering_wrapper)



        # Start the color filtering thread
        self.segmentation_thread = threading.Thread(target=self.color_filtering_thread_func)
        self.segmentation_thread.start()



        # Define image and image array publishers
        self.image_publisher = self.create_publisher(SensorImage, self._color_filter_rgb_image_topic, 10)
        self.get_logger().info(f'[INIT] Merged masks publisher is ready.')


        # Define depth image publisher
        if (self._publish_original_depth_image):
            self.depth_image_publisher = self.create_publisher(SensorImage, self._color_filter_depth_image_topic, 10)
            self.get_logger().info(f'[INIT] Original depth image publisher is ready.')


        # Define depth image camera info publisher
        if (self._publish_original_depth_image_camera_info):
            self.depth_image_camera_info_publisher = self.create_publisher(CameraInfo, self._color_filter_depth_image_camera_info_topic, 10)
            self.get_logger().info(f'[INIT] Original depth image camera info publisher is ready.')

        # Define tf publisher
        if (self._publish_original_tf):
            self.tf_publisher = self.create_publisher(TransformStamped, self._color_filter_tf_topic, 10)
            self.get_logger().info(f"[INIT] TF of the original data's publisher is ready.")


        self.get_logger().info(f'[INIT] Pub-Sub client is ready.')




    def clock_sub(self, msg):
        pass

    """
    To solve the problem of multithreading between clock and segment callbacks, and the discrepancy between the resulting 
    segmented rgb image and the depth, camera and tf messages that needs to have the timestamp of the original image, 
    a possible solution is a subscription to 3 different topics in the `color_filter.py` node. 
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
    (RGB, depth and camera info). For this, the `color_filter.py` node is also listening for the tf of the system. 
    Whenever a tuple of RGB, depth and camera info data is obtained by the callback and processed, in that moment also 
    the tf of the system is saved and published when the segmented image is available, since it is then used in the 
    octomap node to transform in a correct way the data.
    """


    def color_filtering_wrapper(self, rgb_msg, depth_msg, depth_image_camera_info_msg):
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
            self.get_logger().debug(f'[ColorFilter] [color_filtering_wrapper] Before the put the queue size is {self.queue.qsize()}')
            self.queue.put((rgb_msg, depth_msg, depth_image_camera_info_msg, t))
            self.get_logger().debug(f'[ColorFilter] [color_filtering_wrapper] After the put the queue size is {self.queue.qsize()}')
            self.lock = False



    def color_filtering_thread_func(self):
        while rclpy.ok():
            if (self.lock == False):
                # Get the valid data when the queue is filled with it
                self.get_logger().debug(f'[ColorFilter] [color_filtering_thread_func] Before the get the queue size is {self.queue.qsize()}')
                rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg = self.queue.get()
                self.get_logger().debug(f'[ColorFilter] [color_filtering_thread_func] After the get the queue size is {self.queue.qsize()}')
                # Call the actual segmentation callback
                self.color_filtering(rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg)
                self.lock = True






    def color_filtering(self, rgb_msg, depth_msg, depth_image_camera_info_msg, tf_msg):
        
        # Get input data
        self.original_image = rgb_msg
        self.depth_image = depth_msg
        self.depth_image_camera_info = depth_image_camera_info_msg
        self.tf = tf_msg


        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image), cv2.COLOR_BGR2RGB)


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

        if self._colors == "red-green":
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
        filtered_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2RGB)
        filtered_img = np.uint8(filtered_img) # in order to visualize the color image for RViz and also for the exported image
        filtered_img = cv2.convertScaleAbs(filtered_img)
        filtered_img = self.bridge.cv2_to_imgmsg(filtered_img, encoding="rgb8")

        # Add to the header of the output image stamp and frame id of the original rgb image
        start_pub_stamp = self.get_clock().now().to_msg()
        filtered_img.header.frame_id = self.original_image.header.frame_id
        filtered_img.header.stamp = start_pub_stamp



        


        # Publish image
        self.image_publisher.publish(filtered_img)


        # Case when the original depth image needs to be published
        if (self._publish_original_depth_image):
            self.depth_image.header.stamp = start_pub_stamp
            self.get_logger().debug(f'[DepthImage-pub] Publishing original depth image...')
            self.depth_image_publisher.publish(self.depth_image)

        # Case when the original depth image camera info needs to be published
        if (self._publish_original_depth_image_camera_info):
            self.depth_image_camera_info.header.stamp = start_pub_stamp
            self.get_logger().debug(f'[DepthImageCameraInfo-pub] Publishing original depth image camera info...')
            self.depth_image_camera_info_publisher.publish(self.depth_image_camera_info)

        
        # Case when the original tf needs to be published
        if (self._publish_original_tf):
            self.tf.header.stamp = start_pub_stamp
            self.tf_publisher.publish(self.tf)
            self.get_logger().debug('[TF-pub] TF message republished.')



def main(args=None):
    # Init of the node
    rclpy.init(args=args)
    node = ColorFilter()

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