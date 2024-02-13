import os
from typing import List

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from PIL import Image
import matplotlib.pyplot as plt
from lang_sam import LangSAM

from ament_index_python.packages import get_package_share_directory


def display_image_with_boxes(image, boxes, logits):
    fig, ax = plt.subplots()
    ax.imshow(image)
    ax.set_title("Image with Bounding Boxes")
    ax.axis('off')

    for box, logit in zip(boxes, logits):
        x_min, y_min, x_max, y_max = box
        confidence_score = round(logit.item(), 2)  # Convert logit to a scalar before rounding
        box_width = x_max - x_min
        box_height = y_max - y_min

        # Draw bounding box
        rect = plt.Rectangle((x_min, y_min), box_width, box_height, fill=False, edgecolor='red', linewidth=2)
        ax.add_patch(rect)

        # Add confidence score as text
        ax.text(x_min, y_min, f"Confidence: {confidence_score}", fontsize=8, color='red', verticalalignment='top')

    plt.show()



def display_image_with_masks(image, masks):
        num_masks = len(masks)

        fig, axes = plt.subplots(1, num_masks + 1, figsize=(15, 5))
        axes[0].imshow(image)
        axes[0].set_title("Original Image")
        axes[0].axis('off')

        for i, mask_np in enumerate(masks):
            axes[i+1].imshow(mask_np, cmap='gray')
            axes[i+1].set_title(f"Mask {i+1}")
            axes[i+1].axis('off')

        plt.tight_layout()
        plt.show()




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
        image_pil = Image.open(os.path.join(
            get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "tom2.png")).convert("RGB")
        text_prompt = "tomato"
        masks, boxes, phrases, logits = self._lang_sam.predict(image_pil, text_prompt)

        self.get_logger().info(
            f"Found n.{len(masks)} objects referring to the text prompt."
        )

        masks_np = [mask.squeeze().cpu().numpy() for mask in masks]

        display_image_with_masks(image_pil, masks_np)
        display_image_with_boxes(image_pil, boxes, logits)


    



def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    lang_sam_server = LANGSAMServer()
    rclpy.spin(lang_sam_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
