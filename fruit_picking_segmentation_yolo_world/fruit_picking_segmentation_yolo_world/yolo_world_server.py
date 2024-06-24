import rclpy
import rclpy.duration
from rclpy.node import Node

from cv_bridge import CvBridge

import matplotlib.pyplot as plt

import cv2
import torch
import numpy as np

from inference.models.yolo_world.yolo_world import YOLOWorld as YOLOWorld_model
from efficientvit.models.efficientvit.sam import EfficientViTSamPredictor
from efficientvit.sam_model_zoo import create_sam_model
import supervision as sv

from diagnostic_msgs.msg import KeyValue

from fruit_picking_interfaces.msg import ImageArray, Confidence
from fruit_picking_interfaces.srv import YOLOWorldSegmentation



class YOLOWorldServer(Node):

    def __init__(self):
        super().__init__('yolo_world_server')
        
        # Create bridge between cv2 images and sensor_msgs/Image type
        self.bridge = CvBridge()



        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("frame_id", "world"),
                ("publish_masks_array", True),
                ("yolo_world_model_type", "yolo_world/l"),
                ("efficient_SAM_model_type", "l0"),
            ],
        )

        self._frame_id = self.get_parameter("frame_id").value
        self._publish_masks_array = self.get_parameter("publish_masks_array").value
        self._yolo_world_model_type = self.get_parameter("yolo_world_model_type").value
        self._efficient_SAM_model_type = self.get_parameter("efficient_SAM_model_type").value



    
        # Load YOLO World model
        self.get_logger().info(
            f"[INIT] Loading YOLO World model ({self._yolo_world_model_type}). This may take some time..."
        )
        self._yolo_world = YOLOWorld_model(model_id=self._yolo_world_model_type)
        self.get_logger().info("[INIT] YOLO World model loaded.")




        # Load Efficient SAM model and annotators
        efficient_sam_path = f"../michelelagreca/Documents/robotics/models/efficient_SAM_{self._efficient_SAM_model_type}.pt"
        efficient_sam_path_alternative = f"../models/efficient_SAM_{self._efficient_SAM_model_type}.pt"

        self.get_logger().info(
            f"[INIT] Loading Efficient SAM model ({self._efficient_SAM_model_type}) and annotators from folder {efficient_sam_path} This may take some time..."
        )
        self._device = "cuda" if torch.cuda.is_available() else "cpu"
        try:
            self._sam = EfficientViTSamPredictor(
                create_sam_model(name=self._efficient_SAM_model_type, weight_url=efficient_sam_path).to(self._device).eval()
            )
        except Exception as e:
            self.get_logger().warn(f"{e}")           
            self.get_logger().warn(f"[INIT] Failed to load the model. Trying from folder {efficient_sam_path_alternative}")
            try:
                # Attempt with an alternative path or retry the operation
                self._sam = EfficientViTSamPredictor(
                    create_sam_model(name=self._efficient_SAM_model_type, weight_url=efficient_sam_path_alternative).to(self._device).eval()
                )
            except Exception as e:
                self.get_logger().warn(f"{e}")           
                raise RuntimeError("[INIT] Failed to load the model.")

        self._MASK_ANNOTATOR = sv.MaskAnnotator()

        self.get_logger().info("[INIT] Efficient SAM model and annotators loaded.")



        # Starting service
        self.get_logger().info("[INIT] Starting YOLO World server...")

        self._yolo_world_service = self.create_service(
            YOLOWorldSegmentation, "yolo_world_service", self.segment
        )
        self.get_logger().info("[INIT] Yolo World server is ready.")





    def segment(
        self, req: YOLOWorldSegmentation.Request, res: YOLOWorldSegmentation.Response
    ) -> YOLOWorldSegmentation.Response:
        self.get_logger().info('------------------------------------------------')
        self.get_logger().info("[YOLOWorldServer] Received segmentation request.")

        # Get input data
        self.get_logger().info('------------------------------------------------')
        self.get_logger().info('[YOLOWorldServer] Original image received.')
        self.original_image = req.image


        # Format input image from sensor_msgs/Image to cv2 format
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.original_image, "bgr8"), cv2.COLOR_BGR2RGB)
        img_shape = img.shape

        # plt.figure(figsize=(10,10)) 
        # plt.imshow(img)
        # plt.axis('off') 
        # plt.show() 


        # Get prompt
        text_prompt_query = req.text_prompt




        # Segmentation
        self.get_logger().info(
            f"[YOLOWorldServer] Segmenting image of shape {img_shape} with text prompt prior: {text_prompt_query}..."
        )

        # Saving text prompt into an array
        categories = [category.strip() for category in text_prompt_query.split(",")]

        # Set the prompt to YOLO World
        self._yolo_world.set_classes(categories)


        # Inference with Yolo World
        start_detection = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"[YOLOWorldServer] Detection with confidence threshold {req.confidence_threshold} and nms confidence threshold {req.nms_threshold}."
        )
        self.get_logger().info(
            f"[YOLOWorldServer] Detection starting time: {start_detection}."
        )
        results = self._yolo_world.infer(img, confidence=req.confidence_threshold)
        detections = sv.Detections.from_inference(results).with_nms(
            class_agnostic=True, threshold=req.nms_threshold
        )


        # Segmentation with Efficient SAM
        start_segmentation = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"[YOLOWorldServer] Segmentation starting time: {start_segmentation}."
        )
        self._sam.set_image(img, image_format="RGB")
        masks = []
        for xyxy in detections.xyxy:
            mask, _, _ = self._sam.predict(box=xyxy, multimask_output=False)
            masks.append(mask.squeeze())

        self.get_logger().info(
            f"[YOLOWorldServer] Detection (YOLO World) completed in {round((start_segmentation - start_detection)/1.e9, 5)}s."
        )
        self.get_logger().info(
            f"[YOLOWorldServer] Segmentation (Efficient SAM) completed in {round((self.get_clock().now().nanoseconds - start_segmentation)/1.e9, 5)}s."
        )
        self.get_logger().info(
            f"[YOLOWorldServer] Total inference completed in {round((self.get_clock().now().nanoseconds - start_detection)/1.e9, 5)}s."
        )

        detections.mask = np.array(masks)


        # Obtain confidences
        confidences = detections.confidence



        # Start response phase

        # Set the initial time of this phase
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

            # Fill confidences of the response as empty lists
            self.get_logger().debug(f'[YOLOWorldServer] Sending empty confidences...')

            empty_confidences = torch.empty_like(torch.tensor([]))
            empty_confidences = [round(logit.item(), 6) for logit in empty_confidences]
            confidences_dict = dict(zip([], empty_confidences))
            msg_confidences = Confidence()
            msg_confidences.data = [KeyValue(key=k, value=str(v)) for k, v in confidences_dict.items()] 
            msg_confidences.header.frame_id = self.original_image.header.frame_id
            msg_confidences.header.stamp = start_pub_stamp
            msg_confidences.semantic_class = "none"
            res.confidences = msg_confidences
            

            # Fill image of the response with an empty image
            self.get_logger().debug('[YOLOWorldServer] Sending empty image...')

            empty_image = np.ones((img_shape[0], img_shape[1], 3), dtype=np.uint8) * 255
            empty_image = cv2.cvtColor(empty_image, cv2.COLOR_BGR2RGB)
            empty_image = self.bridge.cv2_to_imgmsg(empty_image, encoding="rgb8")
            empty_image.header.frame_id = self.original_image.header.frame_id
            empty_image.header.stamp = start_pub_stamp
            res.merged_masks_images = empty_image


            # Case when the array of masks need to be sent
            self.get_logger().debug('[YOLOWorldServer] Sending empty image array...')
            if (self._publish_masks_array):
            
                # Append the white image inside the images array
                mask_images_array.images.append(empty_image)
                
                 # Publish empty image array
                res.masks_images_array = mask_images_array
                
            
                


        # Case when the model segmented some masks
        else:
            masks_names = [f"mask_{i}" for i in range(1, len(masks) + 1)]

            # Send confidences
            self.get_logger().debug(f'[YOLOWorldServer] Sending confidences...')

            confidences = [round(logit.item(), 6) for logit in confidences]
            confidences_dict = dict(zip(masks_names, confidences))
            msg_confidences = Confidence()
            msg_confidences.data = [KeyValue(key=k, value=str(v)) for k, v in confidences_dict.items()] 
            msg_confidences.header.frame_id = self.original_image.header.frame_id
            msg_confidences.header.stamp = start_pub_stamp
            msg_confidences.semantic_class = text_prompt_query
            res.confidences = msg_confidences

            # Send merged masks 
            self.get_logger().debug('[YOLOWorldServer] Sending merged image...')

            merged_masks_images = np.ones((img.shape[0], img.shape[1], 3), dtype=np.uint8) * 255
            merged_masks_images = cv2.cvtColor(merged_masks_images, cv2.COLOR_BGR2RGB)
            merged_masks_images = self._MASK_ANNOTATOR.annotate(merged_masks_images, detections)
            merged_masks_images = self.bridge.cv2_to_imgmsg(merged_masks_images, encoding="rgb8")
            merged_masks_images.header.frame_id = self.original_image.header.frame_id
            merged_masks_images.header.stamp = start_pub_stamp

            res.merged_masks_images = merged_masks_images

            # Send mage array
            if (self._publish_masks_array):
                self.get_logger().debug('[YOLOWorldServer] Sending image array...')

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
                res.masks_images_array = mask_images_array

        self.get_logger().info(
            f"[YOLOWorldServer] Sending data to the client..."
        )       
        return res





def main(args=None):

    # Init of the node
    rclpy.init(args=args)
    node = YOLOWorldServer()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()