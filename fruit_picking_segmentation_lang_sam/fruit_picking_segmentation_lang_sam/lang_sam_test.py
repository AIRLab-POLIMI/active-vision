import os
from ament_index_python import get_package_share_directory
from typing import List

import rclpy
import rclpy.duration
from rclpy.node import Node

import time

from PIL import Image

import cv2
import numpy as np
import supervision as sv
from lang_sam import LangSAM


from datetime import datetime




class LangSAMNode(Node):

    
    

    def __init__(self, arg=None):
        super().__init__('lang_sam')



        # Parameters
        input_image_name = "flowers_leaves.png"
        text_prompt = "flower"
        model_type = "vit_b"


    

        # Load Lang SAM
        self.get_logger().info(
            f"Loading LANG SAM model '{model_type}'. This may take some time..."
        )

        lang_sam = LangSAM(model_type)



        # Load annotators.
        BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(color=sv.Color(0, 150, 0))
        MASK_ANNOTATOR = sv.MaskAnnotator(color=sv.Color(0, 150, 0))
        LABEL_ANNOTATOR = sv.LabelAnnotator(color=sv.Color(0, 150, 0))

        self.get_logger().info("[INIT] Lang SAM and annotators loaded.")






        try:

            # Load the original image in OpenCV format and visualize
            data_path = os.path.join(
                get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", input_image_name)
            original_image = cv2.imread(data_path)
            self.get_logger().info(
                f"[YOLOWorld] The original image loaded is of type {type(original_image)}."
            )
            cv2.imshow('Original Image', original_image)
            cv2.waitKey(0)  # Wait for a key press to close the window
            cv2.destroyAllWindows()

            # Load the original image as PIL image to be given in SAM input
            original_image_sam = Image.fromarray(original_image)






            # Segmentation
            start_inference = time.perf_counter()
            self.get_logger().info(
                f"[LangSAM] Starting inference..."
            )
            masks, boxes, phrases, logits = lang_sam.predict(original_image_sam, text_prompt)
            
            





            # Times
            end_inference = time.perf_counter()
            inference_time = end_inference - start_inference

            self.get_logger().info(
                f"[LangSAM] Total inference completed in {inference_time:.6f}s."
            )




            # Create detection object to insert the model predictions
            categories = [category.strip() for category in text_prompt.split(",")]
            category_to_id = {category: idx for idx, category in enumerate(categories)}

            # Convert phrases to class IDs using the mapping
            class_ids = np.array([category_to_id[phrase] for phrase in phrases])

            # Convert boxes from PyTorch tensor to NumPy array
            boxes_np = boxes.numpy()

            # Convert masks from PyTorch tensor to NumPy array
            masks_np = masks.numpy()

            # Convert logits from PyTorch tensor to NumPy array
            logits_np = logits.numpy()

            # Create Detections object using the sv library
            detections = sv.Detections(
                xyxy=boxes_np,
                mask=masks_np,
                confidence=logits_np,
                class_id=class_ids
            )

            # Load the segmentation prompt
            labels = [
                f"{categories[class_id]}: {confidence:.4f}"
                for class_id, confidence in zip(detections.class_id, detections.confidence)
            ]   

            

            # Annotation
            output_image = np.array(original_image_sam.copy())

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
            new_image_name = f"lang_sam_{timestamp}_{input_image_name}"

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
    lang_sam_test = LangSAMNode()
    rclpy.spin(lang_sam_test)
    rclpy.shutdown()


if __name__ == "__main__":
    main()




