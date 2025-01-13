# av_segmentation_color_filtering

The `av_segmentation_color_filtering` package is responsible for performing color-based segmentation on images. This package includes source code, launch files, and configuration files required for the color filtering segmentation process.

## Key Components


- **color_filter.launch.py**: This launch file is used to start the color filtering segmentation process. It includes arguments for customizing the launch process, such as setting the topics for RGB and depth images, camera info, and other parameters.


- **color_filter.py**: This source file implements the `color_filter.py` class. The class is responsible for performing color-based segmentation on images. This method uses basic image processing techniques, particularly filtering images based on specified color ranges in the HSV (Hue, Saturation, Value) color space. The primary goal is to isolate regions in an image that match the desired color(s), obtaining a color mask usable in the next steps of the active vision pipeline.

When the input RGB image coming from the sensor is obtained by the Python node responsible for the color-filtering segmentation, a first preprocessing procedure is performed. The result is an image where only the desired color is visible, with all other parts turning black. Then, these black pixels are replaced with white pixels.
