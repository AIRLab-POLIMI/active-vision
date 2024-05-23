import os
from ament_index_python import get_package_share_directory
from typing import List

import rclpy
import rclpy.duration
from rclpy.node import Node

import cv2
import numpy as np
import supervision as sv
import torch
from inference.models.yolo_world.yolo_world import YOLOWorld as YOLOWorld_model

from efficientvit.models.efficientvit.sam import EfficientViTSamPredictor
from efficientvit.sam_model_zoo import create_sam_model

from fruit_picking_segmentation_lang_sam.utils import show_masks_images



class YOLOWorldNode(Node):

    
    

    def __init__(self, arg=None):
        super().__init__('yolo_world')


    
        # Load YOLO World model
        self.get_logger().info(
            f"[INIT] Loading YOLO World model. This may take some time..."
        )

        # Load models.
        yolo_world = YOLOWorld_model(model_id="yolo_world/l")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        sam = EfficientViTSamPredictor(
            create_sam_model(name="l0", weight_url="../models/efficient_SAM_l0.pt").to(device).eval()
        )


        # Load annotators.
        MASK_ANNOTATOR = sv.MaskAnnotator()

        self.get_logger().info("[INIT] YOLO World model loaded.")

      


        try:

            # Load the original image in OpenCV format
            original_image_path = os.path.join(
                get_package_share_directory("fruit_picking_segmentation_yolo_world"), "data", "tomatoes.jpeg")
            original_image = cv2.imread(original_image_path)
            original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
            self.get_logger().info(
                f"[YOLOWorld] The original image loaded is of type {type(original_image)}."
            )


            # Load the segmentation prompt
            text_prompt = "tomatoes"
            categories = [category.strip() for category in text_prompt.split(",")]



            # Set the prompt to YOLO World
            yolo_world.set_classes(categories)
            self.get_logger().info(
                f"[YOLOWorld] Prompt: {categories}. Starting segmentation..."
            )


            # Inference with Yolo World
            start_inference = self.get_clock().now().nanoseconds

            results = yolo_world.infer(original_image, confidence=0.0005)
            detections = sv.Detections.from_inference(results).with_nms(
                class_agnostic=True, threshold=0.2
            )
            self.get_logger().info(
                f"[YOLOWorld] Object detections: {detections}."
            )


            # Segmentation.
            sam.set_image(original_image, image_format="RGB")
            masks = []
            for xyxy in detections.xyxy:
                mask, _, _ = sam.predict(box=xyxy, multimask_output=False)
                masks.append(mask.squeeze())

            self.get_logger().info(
                f"[YOLOWorld] Inference completed in {round((self.get_clock().now().nanoseconds - start_inference)/1.e9, 5)}s."
            )

            detections.mask = np.array(masks)

            # Obtain labels
            labels = [
                f"{categories[class_id]}: {confidence:.2f}"
                for class_id, confidence in zip(detections.class_id, detections.confidence)
            ]

            # Create empty image
            empty_image = np.ones((original_image.shape[0], original_image.shape[1], 3), dtype=np.uint8) * 255
            empty_image = cv2.cvtColor(empty_image, cv2.COLOR_BGR2RGB)

            empty_image = MASK_ANNOTATOR.annotate(empty_image, detections)

            # # Create single masks
            # single_masks = []
            # for i in range(len(detections.xyxy)):
            #     single_detection = sv.Detections.empty()
            #     single_detection.xyxy = np.array([detections.xyxy[i]])
            #     single_detection.mask = np.array([masks[i]])
            #     single_detection.confidence = np.array([detections.confidence[i]])
            #     single_detection.class_id = np.array([detections.class_id[i]])
            #     single_detection.tracker_id = None
            #     single_detection.data = {'class_name': np.array([detections.data['class_name'][i]])}

            #     self.get_logger().info(
            #         f"[YOLOWorld] Object detections: {single_detection}."
            #     )

            #     final_mask = np.ones((original_image.shape[0], original_image.shape[1], 3), dtype=np.uint8) * 255
            #     final_mask = cv2.cvtColor(final_mask, cv2.COLOR_BGR2RGB)

            #     final_mask = MASK_ANNOTATOR.annotate(final_mask, single_detection)
            #     single_masks.append(final_mask)



            # Plot masks images
            self.get_logger().info("Plotting original image...") 

            show_masks_images(original_image, []) 
            show_masks_images(empty_image, []) 

            # for single_mask in single_masks:
            #     show_masks_images(single_mask, [])

            self.get_logger().info("Original image plotted.")

            

            
        except Exception as e:
            err_msg = f"Failure during service call. Full message: {e}."
            self.get_logger().error(err_msg)
            raise RuntimeError(err_msg)





def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    yolo_world_test = YOLOWorldNode()
    rclpy.spin(yolo_world_test)
    rclpy.shutdown()


if __name__ == "__main__":
    main()




