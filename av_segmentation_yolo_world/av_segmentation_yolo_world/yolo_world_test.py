import os
from ament_index_python import get_package_share_directory
from typing import List

import rclpy
import rclpy.duration
from rclpy.node import Node

import time

import cv2
import numpy as np
import supervision as sv
import torch
from inference.models.yolo_world.yolo_world import YOLOWorld as YOLOWorld_model

from efficientvit.models.efficientvit.sam import EfficientViTSamPredictor
from efficientvit.sam_model_zoo import create_sam_model

from datetime import datetime



class YOLOWorldNode(Node):

    
    

    def __init__(self, arg=None):
        super().__init__('yolo_world')



        # Parameters
        efficient_SAM_model_type = "l0"
        yolo_world_model_type = "yolo_world/l"
        input_image_name = "flowers_leaves.png"
        text_prompt = "flower"
        confidence_threshold = 0.01
        nms_threshold = 0.5



    
        # Load Efficient SAM model and annotators
        efficient_sam_path = f"../michelelagreca/Documents/robotics/models/efficient_SAM_{efficient_SAM_model_type}.pt"
        efficient_sam_path_alternative = f"../models/efficient_SAM_{efficient_SAM_model_type}.pt"

        self.get_logger().info(
            f"[INIT] Loading Efficient SAM model ({efficient_SAM_model_type}) and annotators from folder {efficient_sam_path} This may take some time..."
        )
        device = "cuda" if torch.cuda.is_available() else "cpu"
        try:
            sam = EfficientViTSamPredictor(
                create_sam_model(name=efficient_SAM_model_type, weight_url=efficient_sam_path).to(device).eval()
            )
        except Exception as e:
            self.get_logger().warn(f"{e}")           
            self.get_logger().warn(f"[INIT] Failed to load the model. Trying from folder {efficient_sam_path_alternative}")
            try:
                # Attempt with an alternative path or retry the operation
                sam = EfficientViTSamPredictor(
                    create_sam_model(name=efficient_SAM_model_type, weight_url=efficient_sam_path_alternative).to(device).eval()
                )
            except Exception as e:
                self.get_logger().warn(f"{e}")           
                raise RuntimeError("[INIT] Failed to load the model.")

        # Load annotators.
        BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator()
        MASK_ANNOTATOR = sv.MaskAnnotator()
        LABEL_ANNOTATOR = sv.LabelAnnotator()

        self.get_logger().info("[INIT] Efficient SAM model and annotators loaded.")




    
        # Load YOLO World model
        self.get_logger().info(
            f"[INIT] Loading YOLO World model ({yolo_world_model_type}). This may take some time..."
        )
        yolo_world = YOLOWorld_model(model_id=yolo_world_model_type)
        self.get_logger().info("[INIT] YOLO World model loaded.")

      




        try:

            # Load the original image in OpenCV format and visualize
            data_path = os.path.join(
                get_package_share_directory("av_segmentation_yolo_world"), "data", input_image_name)
            original_image = cv2.imread(data_path)
            self.get_logger().info(
                f"[YOLOWorld] The original image loaded is of type {type(original_image)}."
            )
            cv2.imshow('Original Image', original_image)
            cv2.waitKey(0)  # Wait for a key press to close the window
            cv2.destroyAllWindows()






            # Load the segmentation prompt
            categories = [category.strip() for category in text_prompt.split(",")]



            # Set the prompt to YOLO World
            yolo_world.set_classes(categories)
            self.get_logger().info(
                f"[YOLOWorld] Prompt: {categories}. Starting segmentation..."
            )


            # Inference with Yolo World
            start_inference_yolo_world = time.perf_counter()

            results = yolo_world.infer(original_image, confidence=confidence_threshold)
            detections = sv.Detections.from_inference(results).with_nms(
                class_agnostic=True, threshold=nms_threshold
            )
            self.get_logger().info(
                f"[YOLOWorld] Object detections: {detections}."
            )


            # Segmentation.
            self.get_logger().info(
                f"[YOLOWorld] Starting segmentation..."
            )
            start_inference_efficient_sam = time.perf_counter()
            sam.set_image(original_image, image_format="RGB")
            masks = []
            for xyxy in detections.xyxy:
                mask, _, _ = sam.predict(box=xyxy, multimask_output=False)
                masks.append(mask.squeeze())
            detections.mask = np.array(masks)




            # Times
            final_time = time.perf_counter()
            yolo_world_inference_time = start_inference_efficient_sam - start_inference_yolo_world
            efficient_sam_inference_time = final_time - start_inference_efficient_sam
            total_inference_time = final_time - start_inference_yolo_world

            self.get_logger().info(
                f"[YOLOWorld] YOLO World inference completed in {yolo_world_inference_time:.6f}s."
            )
            self.get_logger().info(
                f"[YOLOWorld] Efficient SAM inference completed in {efficient_sam_inference_time:.6f}s."
            )
            self.get_logger().info(
                f"[YOLOWorld] Total inference completed in {total_inference_time:.6f}s."
            )

            



            # Annotation
            output_image = original_image
            # output_image = cv2.convertScaleAbs(original_image, alpha=1.0, beta=50) # to increse the brightness
            labels = [
                f"{categories[class_id]}: {confidence:.4f}"
                for class_id, confidence in zip(detections.class_id, detections.confidence)
            ]
            output_image = MASK_ANNOTATOR.annotate(output_image, detections)
            output_image = BOUNDING_BOX_ANNOTATOR.annotate(output_image, detections)
            output_image = LABEL_ANNOTATOR.annotate(output_image, detections, labels=labels)





            # Plot masks images
            self.get_logger().info("Plotting segmented image...") 

            cv2.imshow('Segmented Image', output_image)
            cv2.waitKey(0)  # Wait for a key press to close the window
            cv2.destroyAllWindows()

            self.get_logger().info("Segmented image plotted.")



            # Save image
            timestamp = datetime.now().strftime("%y%m%d%H%M%S")
            new_image_name = f"yolo_world_{timestamp}_{input_image_name}"

            self.get_logger().info(f"Saving segmented image {new_image_name}...") 

            new_image_path = os.path.join(os.path.dirname(data_path), new_image_name)
            cv2.imwrite(new_image_path, output_image)

            self.get_logger().info(f"{new_image_name} segmented image saved.") 

            

            
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




