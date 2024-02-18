import os
from typing import List

import cv2
import numpy as np
import rclpy
from ament_index_python import get_package_share_directory

from typing import List, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from fruit_picking_interfaces.srv import LANGSAMSegmentation

from fruit_picking_segmentation_lang_sam.utils import (
    merge_masks_images,
    show_masks_images,
    show_masks_images_with_confidence,
    show_boxes_with_confidence,
    export_masks_images,
    export_merged_masks_images
)


class LANGSAMClient(Node):
    """
    Client for the LANG SAM segmentation service
    It send a test image to the service, wait for the response containing
    the results of the inference of the image and text prompt, and plot, export and print the reuslting masks
    and confidences and bounding boxes
    """


    def __init__(
        self, node_name: str = "lang_sam_client", service_name: str = "/segmentation_lang_sam"
    ) -> None:
        

        """Initialize connection to the LANG SAM segmentation service
        Args:
            node_name (string): Node name, defaults to 'lang_sam_client'.
            service_name (string): Service name, defaults to 'segmentation_lang_sam'.
        """


        super().__init__(node_name)
        self._bridge = CvBridge()


        self._lang_sam_segment_client = self.create_client(
            LANGSAMSegmentation, f"{service_name}"
        )
        while not self._lang_sam_segment_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().info(
                    "Interrupted while waiting for service. Exiting."
                )
                return
            self.get_logger().info(
                f"Waiting for '{service_name}' service to become available..."
            )


        self.get_logger().info(f"Established client for '{service_name}' service.")





    def sync_segment_request(
        self,
    ) -> Tuple[List[np.ndarray], List[float], List[float], np.ndarray, str]:
        

        """Performs a synchronous (blocking) segmentation request to ROS 2 LANG SAM server.
        Takes the input image and the input text prompts.
    
        Args:
            img_rgb (np.ndarray, or cv2 format (same)): RGB image that will be segmented
            text prompt (string): string containing the description of what the model have to segment in the query image
        Returns:
            Masks (List(np.ndarray or cv2 format (same))): Segmentation masks of the segmented object.
            Boxes (List(float)): 4 values for each mask containin the bounds of the mask in the image.
            Confidences (List(float)): Confidence scores of the segmentation masks.
        """


        # Load parameters from launch file
        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_name", "car.jpeg"),
                ("prompt", "wheels"),
            ],
        )

        self._image_name = self.get_parameter("image_name").value
        self._prompt = self.get_parameter("prompt").value



        # Load the original image in OpenCV format
        original_image = cv2.imread(os.path.join(
            get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", self._image_name))
        original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

        text_prompt = self._prompt



        future = self._lang_sam_segment_client.call_async(
            LANGSAMSegmentation.Request(
                image=self._bridge.cv2_to_imgmsg(original_image),
                text_prompt=text_prompt,
            )
        )

        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError("Segmentation service call failed")
        res = future.result()
        return [self._bridge.imgmsg_to_cv2(mask_image) for mask_image in res.masks_images], res.boxes, res.confidences, original_image, text_prompt





def main(args: List[str] = None) -> None:
    rclpy.init(args=args)

    lang_sam_client = LANGSAMClient(
        node_name="lang_sam_client",
        service_name="segmentation_lang_sam",
    )

    try:


        # Result send from the server
        masks_images, boxes, confidences, original_image, text_prompt = lang_sam_client.sync_segment_request()

        print(
            f"Response received from the server."
        )
        print(
            f"Found n.{len(masks_images)} objects related to the prompt: '{text_prompt}'."
        )


        if len(masks_images) <= 0:

            # Plot masks images
            print("Plotting original image...") 

            show_masks_images(original_image, masks_images) 

            print("Original image plotted.")


        else:   

            # Convert boxes tensor in list of lists
            boxes = np.array(boxes.data).reshape((len(boxes.data) // 4, 4))
            boxes = boxes.tolist()
        


            # Print results

            # print(
            #     f"Type of masks: {type(masks_images)}."
            # )

            # print(f"The boxes list: {boxes} of type {type(boxes)}")
            # print(f"The confidences list: {confidences} of type {type(confidences)}")

            # for mask_image, box, confidence in zip(masks_images, boxes, confidences):
            #     print(
            #         f"Mask: \n{mask_image}. Type of this data: {type(mask_image)}"
            #     )
            #     print(
            #         f"Box of mask: \n{box}. Type of this data: {type(box)}."
            #     )
            #     print(
            #         f"Confidence of mask: \n{confidence}. Type of this data: {type(confidences)}"
            #     )
            # print(
            #         f"Type of original image data: {type(original_image)}. image: \n{original_image}"
            #     )
        

            # Plot masks images
            print("Plotting masks images...") 

            show_masks_images(original_image, masks_images) 

            print("Masks images plotted.")




            # Plot merged masks images
            # print("Plotting merged masks images...") 

            # merged_masks_images = merge_masks_images(masks_images)
            # show_masks_images(original_image, np.array([merged_masks_images])) 
            # print(
            #     f"Merged (cv2 image): \n{merged_masks_images}. Type of this data: {type(merged_masks_images)}"
            # )

            # print("Masks merged images plotted.")




            # Plotting image with masks and confidences
            print("Plotting masks images with confidence...") 

            show_masks_images_with_confidence(original_image, masks_images, boxes, confidences) 

            print("Masks images with confidence plotted.")




            # Plotting image with bounding boxes and confidences
            print("Plotting bounding boxes with confidence...") 

            show_boxes_with_confidence(original_image, boxes, confidences) 

            print("Bounding boxes with confidence plotted.")





            # # Export masks images
            # print("Exporting masks images...") 

            # export_masks_images(masks_images, os.path.join(
            #     get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

            # print("Masks images exported.")




            # # Export merged masks images
            # print("Exporting merged masks images...") 

            # export_merged_masks_images(merged_masks_images, os.path.join(
            #     get_package_share_directory("fruit_picking_segmentation_lang_sam"), "data", "masks"))

            # print("Merged masks images exported.")


        print(
            f"Client terminated."
        )

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
