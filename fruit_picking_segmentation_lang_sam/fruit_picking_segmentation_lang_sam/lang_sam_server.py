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

from fruit_picking_segmentation_lang_sam.test import (
    test_segmentation, 
    fake_segmentation,
    )


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
        test_segmentation(self)
        self.get_logger().info("Test segmentation completed.")







def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    lang_sam_server = LANGSAMServer()
    rclpy.spin(lang_sam_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
