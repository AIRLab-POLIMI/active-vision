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
                ("base_frame_id", "igus_rebel_base_link"), 
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
        self._base_frame_id = self.get_parameter("base_frame_id").value
        self._publish_original_depth_image = self.get_parameter("publish_original_depth_image").value
        self._publish_original_depth_image_camera_info = self.get_parameter("publish_original_depth_image_camera_info").value
        self._publish_original_tf = self.get_parameter("publish_original_tf").value
        self._colors = self.get_parameter("colors").value  




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



    def color_filtering_wrapper(self, rgb_msg, depth_msg, depth_image_camera_info_msg):
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