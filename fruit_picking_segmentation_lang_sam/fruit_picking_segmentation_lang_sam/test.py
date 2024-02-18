import os
from typing import List
import torch
from PIL import Image
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

from fruit_picking_segmentation_lang_sam.utils import (
    convert_masks_to_images, 
    merge_masks_images,
    show_masks_images,
    show_masks_images_with_confidence,
    show_boxes_with_confidence,
    export_masks_images,
    export_merged_masks_images
)


def test_segmentation(self):

    # Load the original image in OpenCV format
    original_image = cv2.imread(os.path.join(
        get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "car.jpeg"))
    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
    # Load the original image as PIL image to be given in SAM input
    original_image_sam = Image.fromarray(original_image)

    text_prompt = "wheel"


    # Predict from LANG SAM
    masks, boxes, phrases, logits = self._lang_sam.predict(original_image_sam, text_prompt)
    # Convert confidences tensor in list fo float rounded at the 3rd digit
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
    


    # Convert bool masks to cv2 images
    masks_images = convert_masks_to_images(masks)
    for mask_image in masks_images:
        self.get_logger().info(
            f"Mask (cv2 image): \n{mask_image}. Type of this data: {type(mask_image)}"
        )
    # Convert boxes tensor in list of lists
    # boxes = boxes.tolist()


    # Different types of boxes
    self.get_logger().info(
        f"\nDifferent types of boxes:\n"
    )
    self.get_logger().info(
        f"Boxes (tensor): \n{boxes}. Type of this data: {type(boxes)}"
    )
    self.get_logger().info(
        f"Boxes (list of lists): \n{boxes.tolist()}. Type of this data: {type(boxes.tolist())}"
    )
    self.get_logger().info(
        f"Boxes (np.array): \n{boxes.squeeze().cpu().numpy()}. Type of this data: {type(boxes.squeeze().cpu().numpy())}"
    )
    self.get_logger().info(
        f"Boxes (list): \n{boxes.squeeze().cpu().numpy().astype(float).ravel()}. Type of this data: {type(boxes.squeeze().cpu().numpy().astype(float).ravel())}"
    )



    


    # Plot masks images
    self.get_logger().info("Plotting masks images...") 

    show_masks_images(original_image, masks_images) 

    self.get_logger().info("Masks images plotted.")




    # Plot merged masks images
    self.get_logger().info("Plotting merged masks images...") 

    merged_masks_images = merge_masks_images(masks_images)
    show_masks_images(original_image, np.array([merged_masks_images])) 
    self.get_logger().info(
        f"Merged (cv2 image): \n{merged_masks_images}. Type of this data: {type(merged_masks_images)}"
    )

    self.get_logger().info("Masks merged images plotted.")




    # Plotting image with masks and confidences
    self.get_logger().info("Plotting masks images with confidence...") 

    show_masks_images_with_confidence(original_image, masks_images, boxes, confidences) 

    self.get_logger().info("Masks images with confidence plotted.")




    # Plotting image with bounding boxes and confidences
    self.get_logger().info("Plotting bounding boxes with confidence...") 

    show_boxes_with_confidence(original_image, boxes, confidences) 

    self.get_logger().info("Bounding boxes with confidence plotted.")





    # Export masks images
    self.get_logger().info("Exporting masks images...") 

    export_masks_images(masks_images, os.path.join(
        get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

    self.get_logger().info("Masks images exported.")




    # Export merged masks images
    self.get_logger().info("Exporting merged masks images...") 

    export_merged_masks_images(merged_masks_images, os.path.join(
        get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

    self.get_logger().info("Merged masks images exported.")





def fake_segmentation(self):
    
    # Load the original image in OpenCV format
    original_image = cv2.imread(os.path.join(
        get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "car.jpeg"))
    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
    original_height, original_width, channels = original_image.shape
    


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
    


    # Convert bool masks to cv2 images
    masks_images = convert_masks_to_images(masks)
    for mask_image in masks_images:
        self.get_logger().info(
            f"Mask (cv2 image): \n{mask_image}. Type of this data: {type(mask_image)}"
        )
    # Convert boxes tensor in list of lists
    boxes = boxes.tolist()
    # Convert confidence in list
    confidences = confidences.tolist()



    


    # Plot masks images
    self.get_logger().info("Plotting masks images...") 

    show_masks_images(original_image, masks_images) 

    self.get_logger().info("Masks images plotted.")




    # Plot merged masks images
    self.get_logger().info("Plotting merged masks images...") 

    merged_masks_images = merge_masks_images(masks_images)
    show_masks_images(original_image, np.array([merged_masks_images])) 
    self.get_logger().info(
        f"Merged (cv2 image): \n{merged_masks_images}. Type of this data: {type(merged_masks_images)}"
    )

    self.get_logger().info("Masks merged images plotted.")




    # Plotting image with masks and confidences
    self.get_logger().info("Plotting masks images with confidence...") 

    show_masks_images_with_confidence(original_image, masks_images, boxes, confidences) 

    self.get_logger().info("Masks images with confidence plotted.")




    # Plotting image with bounding boxes and confidences
    self.get_logger().info("Plotting bounding boxes with confidence...") 

    show_boxes_with_confidence(original_image, boxes, confidences) 

    self.get_logger().info("Bounding boxes with confidence plotted.")





    # Export masks images
    self.get_logger().info("Exporting masks images...") 

    export_masks_images(masks_images, os.path.join(
        get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

    self.get_logger().info("Masks images exported.")




    # Export merged masks images
    self.get_logger().info("Exporting merged masks images...") 

    export_merged_masks_images(merged_masks_images, os.path.join(
        get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

    self.get_logger().info("Merged masks images exported.")


