import os
from typing import List

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import torch

from PIL import Image
import matplotlib.pyplot as plt
from lang_sam import LangSAM
import random
import torchvision.transforms as T

from ament_index_python.packages import get_package_share_directory


from fruit_picking_segmentation_lang_sam.utils import (
    convert_masks_to_images, 
    export_masks_images,
    merge_masks_images,
    show_masks_images,
    show_masks_images_with_confidence,
    show_boxes_with_confidence)




class LANGSAMServer(Node):
    def __init__(self, node_name: str = "lang_sam_server") -> None:
        super().__init__(node_name)
        self.get_logger().info("Starting SAM server...")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("model_type", "vit_h"),
            ],
        )
        self._bridge = CvBridge()
        self._model_type = self.get_parameter("model_type").value

        self.get_logger().info(
            f"Loading SAM model '{self._model_type}'. This may take some time..."
        )

        self._lang_sam = LangSAM(
            self._model_type,
        )

        self.get_logger().info("SAM model loaded.")


        self.get_logger().info("Starting test segmentation...")
        self.test_segmentation()
        self.get_logger().info("Test segmentation completed.")




    def test_segmentation(self):

        # Get original image and text prompts
        original_image = Image.open(os.path.join(
            get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "car.jpeg")).convert("RGB")
        text_prompt = "wheel"


        # Predict from LANG SAM
        masks, boxes, phrases, logits = self._lang_sam.predict(original_image, text_prompt)
        confidences = [round(logit.item(), 3) for logit in logits]

        

        # Print results
        self.get_logger().info(
            f"Found n.{len(masks)} objects referring to the text prompt. Type of masks: {type(masks)}. Size of masks: {masks.size()}."
        )
        self.get_logger().info(f"The size of the boxes tensor is {boxes.size()}. The tensor: {boxes}")
        for mask, box, confidence in zip(masks, boxes, confidences):
            self.get_logger().info(
                f"Mask: \n{mask}. Type of this data: {type(mask)}"
            )
            self.get_logger().info(
                f"Box of mask: \n{box}. Type of this data: {type(box)}. Size: {box.size()}"
            )
            self.get_logger().info(
                f"Confidence of mask: \n{confidence}. Type of this data: {type(confidences)}"
            )
        self.get_logger().info(
                f"Type of original image data: {type(original_image)}. image: \n{original_image}"
            )
        


        # Convert bool masks to PIL images
        masks_images = convert_masks_to_images(masks)
        for mask_image in masks_images:
            self.get_logger().info(
                f"Mask (image): \n{mask_image}. Type of this data: {type(mask_image)}"
            )
        # Convert boxes tensor in list of lists
        boxes = boxes.tolist()



        # Export masks images
        # self.get_logger().info("Exporting masks images...") 

        # export_masks_images(masks_images, os.path.join(
        #     get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

        # self.get_logger().info("Masks images exported.")



        # Plot masks images
        self.get_logger().info("Plotting masks images...") 

        show_masks_images(original_image, masks_images) 

        self.get_logger().info("Masks images plotted.")




        # Plot and export merged masks images
        # self.get_logger().info("Plotting and exporting merged masks images...") 

        # merged_masks_images = merge_masks_images(masks_images)
        # show_masks_images(original_image, [merged_masks_images]) 
        # export_masks_images([merged_masks_images], os.path.join(
        #     get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

        # self.get_logger().info("Masks merged images plotted and exported.")




        # Plotting image with masks and confidences
        self.get_logger().info("Plotting masks images with confidence...") 

        show_masks_images_with_confidence(original_image, masks_images, boxes, confidences) 

        self.get_logger().info("Masks images with confidence plotted.")


        # Plotting image with bounding boxes and confidences
        self.get_logger().info("Plotting bounding boxes with confidence...") 

        show_boxes_with_confidence(original_image, boxes, confidences) 

        self.get_logger().info("Bounding boxes with confidence plotted.")


    

    
    def fake_segmentation(self):
        
        # Load the original image
        original_image = Image.open(os.path.join(
            get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "car.jpeg")).convert("RGB")
        original_width, original_height = original_image.size
        


        # Create fake tensors of masks

        # Define the size of the circle
        radius = 100  # Adjust this value to change the circle size

        # Define the image size
        image_size = (original_height, original_width)

        # Create a 4D tensor filled with zeros
        masks = torch.zeros(4, *image_size, dtype=torch.bool)

        # Function to generate a random circle mask
        def generate_circle_mask(image_size, radius):
            center_x = torch.randint(0, image_size[0], size=(1,)).item()
            center_y = torch.randint(0, image_size[1], size=(1,)).item()
            grid_x, grid_y = torch.meshgrid(torch.arange(image_size[0]), torch.arange(image_size[1]))
            distance = torch.sqrt(((grid_x - center_x) ** 2) + ((grid_y - center_y) ** 2))
            return distance < radius

        # Generate random circle masks for each 2D tensor
        for i in range(4):
            mask = generate_circle_mask(image_size, radius)
            masks[i] = mask




        # Create fake tensor of boxes bounds
        boxes = torch.zeros(4, 4, dtype=torch.int)

        # Compute coordinates for each image
        for i in range(4):
            indices = torch.nonzero(masks[i])
            if indices.size(0) == 0:
                boxes[i] = torch.tensor([0, 0, 0, 0])
            else:
                ymin = indices[:, 0].min().item()
                ymax = indices[:, 0].max().item()
                xmin = indices[:, 1].min().item()
                xmax = indices[:, 1].max().item()
                boxes[i] = torch.tensor([xmin, ymin, xmax, ymax])
        


        # create fake tensor of confidences
        confidences = torch.tensor([0.7, 0.8, 0.6, 0.3])





        # Print results
        self.get_logger().info(
            f"Found n.{len(masks)} objects referring to the text prompt. Type of masks: {type(masks)}. Size of masks: {masks.size()}."
        )
        self.get_logger().info(f"The size of the boxes tensor is {boxes.size()}. The tensor: {boxes}")
        for mask, box, confidence in zip(masks, boxes, confidences):
            self.get_logger().info(
                f"Mask: \n{mask}. Type of this data: {type(mask)}"
            )
            self.get_logger().info(
                f"Box of mask: \n{box}. Type of this data: {type(box)}. Size: {box.size()}"
            )
            self.get_logger().info(
                f"Confidence of mask: \n{confidence}. Type of this data: {type(confidences)}"
            )
        self.get_logger().info(
                f"Type of original image data: {type(original_image)}. image: \n{original_image}"
            )
        


        # Convert bool masks to PIL images
        masks_images = convert_masks_to_images(masks)
        for mask_image in masks_images:
            self.get_logger().info(
                f"Mask (image): \n{mask_image}. Type of this data: {type(mask_image)}"
            )
        # Convert boxes tensor in list of lists
        boxes = boxes.tolist()
        # Convert confidence in list
        confidences = confidences.tolist()



        # Export masks images
        # self.get_logger().info("Exporting masks images...") 

        # export_masks_images(masks_images, os.path.join(
        #     get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

        # self.get_logger().info("Masks images exported.")



        # Plot masks images
        self.get_logger().info("Plotting masks images...") 

        show_masks_images(original_image, masks_images) 

        self.get_logger().info("Masks images plotted.")




        # Plot and export merged masks images
        # self.get_logger().info("Plotting and exporting merged masks images...") 

        # merged_masks_images = merge_masks_images(masks_images)
        # show_masks_images(original_image, [merged_masks_images]) 
        # export_masks_images([merged_masks_images], os.path.join(
        #     get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

        # self.get_logger().info("Masks merged images plotted and exported.")




        # Plotting image with masks and confidences
        self.get_logger().info("Plotting masks images with confidence...") 

        show_masks_images_with_confidence(original_image, masks_images, boxes, confidences) 

        self.get_logger().info("Masks images with confidence plotted.")


        # Plotting image with bounding boxes and confidences
        self.get_logger().info("Plotting bounding boxes with confidence...") 

        show_boxes_with_confidence(original_image, boxes, confidences) 

        self.get_logger().info("Bounding boxes with confidence plotted.")

    




def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    lang_sam_server = LANGSAMServer()
    rclpy.spin(lang_sam_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
