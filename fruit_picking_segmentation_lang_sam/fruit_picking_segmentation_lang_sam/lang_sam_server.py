from typing import List

import cv2
from PIL import Image
from lang_sam import LangSAM

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from fruit_picking_segmentation_lang_sam.utils import convert_masks_to_images
    
from fruit_picking_segmentation_lang_sam.test import test_segmentation

from fruit_picking_interfaces.srv import LANGSAMSegmentation



class LANGSAMServer(Node):

    '''
    The initialization of the LANG SAM server consist in create a Node of the class LANGSAMServer.
    This node firstly create a model, downloading if it is not already present in the system.
    
    '''
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
            f"Loading LANG SAM model '{self._model_type}'. This may take some time..."
        )

        self._lang_sam = LangSAM(
            self._model_type,
        )

        self.get_logger().info("LANG SAM model loaded.")


        # self.get_logger().info("Starting test segmentation...")
        # test_segmentation(self)
        # self.get_logger().info("Test segmentation completed.")

        self.get_logger().info("Starting LANG SAM server...")

        self._sam_segment_service = self.create_service(
            LANGSAMSegmentation, "/segmentation_lang_sam", self._segmentation_lang_sam
        )
        self.get_logger().info("LANG SAM server is ready.")


    def _segmentation_lang_sam(
        self, req: LANGSAMSegmentation.Request, res: LANGSAMSegmentation.Response
    ) -> LANGSAMSegmentation.Response:
        self.get_logger().info("Received segmentation request.")
        try:

            # Request data
            img = cv2.cvtColor(self._bridge.imgmsg_to_cv2(req.image), cv2.COLOR_BGR2RGB)
            # Convert the image in PIL format to be given in input to the model
            img_query = Image.fromarray(img)
            text_prompt_query = req.text_prompt



            # Segmentation
            self.get_logger().info(
                f"Segmenting image of shape {img.shape} with text prompt prior: {text_prompt_query}"
            )
            start = self.get_clock().now().nanoseconds

            # Predict from LANG SAM
            masks, boxes, phrases, logits = self._lang_sam.predict(img_query, text_prompt_query)

            self.get_logger().info(
                f"Segmentation completed in {round((self.get_clock().now().nanoseconds - start)/1.e9, 2)}s."
            )

            # Check number of masks found
            self.get_logger().info(
                f"Masks found: {masks.size(0)}."
            )


            # Create response with model results

            # Convert bool masks tensor to cv2 images
            masks_images = convert_masks_to_images(masks)
            
            # Convert boxes tensor in std_msgs/Float64MultiArray
            boxes = boxes.squeeze().cpu().numpy().astype(float).ravel().tolist()
            msg_boxes = Float64MultiArray()
            msg_boxes.data = boxes

            # Convert confidences tensor in list fo float rounded at the 3rd digit
            confidences = [round(logit.item(), 3) for logit in logits]
        

            # Create response
            res.masks_images = [self._bridge.cv2_to_imgmsg(mask_image) for mask_image in masks_images]
            res.boxes = msg_boxes
            res.confidences = confidences

            self.get_logger().info(
                f"Response sent to the client."
            )
            
            return res

            
        except Exception as e:
            err_msg = f"Failure during service call. Full message: {e}."
            self.get_logger().error(err_msg)
            raise RuntimeError(err_msg)





def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    lang_sam_server = LANGSAMServer()
    rclpy.spin(lang_sam_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()



