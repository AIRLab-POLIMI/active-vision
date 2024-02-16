import os
from typing import List
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import cv2



def convert_masks_to_images(masks, random_color=True): 
    images = []
    for mask in masks: 
        if random_color: 
            color = np.concatenate([np.random.random(3), np.array([1.0])], axis=0) 
        else: 
            color = np.array([0/255, 0/255, 255/255, 1.0]) 
        h = mask.size(dim=0)
        w = mask.size(dim=1)
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1) 
        image = mask_image.squeeze().cpu().numpy()
        images.append(image)
    return images



def merge_masks_images(masks_images, overlapping=True):
    if overlapping: 
    
        # Create a blank image with the same size and mode as the first image
        base_image = np.zeros_like(masks_images[0])

        # Composite each image onto the base image
        for mask_image in masks_images:
            # Replace non-transparent pixels of the base image with corresponding pixels from the current image
            non_transparent_pixels = mask_image[:, :, 3] > 0
            base_image[non_transparent_pixels] = mask_image[non_transparent_pixels]
    else:
        # Create a blank image with the same size and mode as the first image
        base_image = np.zeros_like(masks_images[0])

        # Composite each image onto the base image
        for mask_image in masks_images:
            base_image = cv2.addWeighted(base_image, 1, mask_image, 1, 0)
        
        ## If the merged images has to have an alpha channel smaller that 1:
        # b, g, r, a = cv2.split(base_image)

        # new_alpha_value = 0.8
        # a[:] = new_alpha_value

        # # Merge the channels back together
        # base_image = cv2.merge((b, g, r, a))

    return base_image



def show_masks_images(original_image, masks_images): 
    plt.figure(figsize=(10,10)) 
    plt.imshow(original_image)
    for mask_image in masks_images: 
        plt.gca().imshow(mask_image)
    plt.axis('off') 
    plt.show() 



def show_masks_images_with_confidence(original_image, masks_images, boxes, confidences):
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    # Plot original image
    axes[0].imshow(original_image)
    axes[0].set_title('Original Image')
    axes[0].axis('off')

    # Plot original image with masks and confidences
    axes[1].imshow(original_image)
    axes[1].set_title('Original Image with Masks')
    axes[1].axis('off')

    for mask_img, box, confidence in zip(masks_images, boxes, confidences):
        mask_array = np.array(mask_img)

        # Overlay mask on the image
        axes[1].imshow(mask_array, alpha=1)

        # Calculate position for confidence text
        x_pos = (box[0] + box[2]) / 2
        y_pos = (box[1] + box[3]) / 2

        # Plot confidence text near the mask
        axes[1].text(x_pos, y_pos, f'Conf: {confidence:.2f}', color='red')

    plt.tight_layout()
    plt.show()



def show_boxes_with_confidence(original_image, boxes, confidences):
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    # Plot original image
    axes[0].imshow(original_image)
    axes[0].set_title('Original Image')
    axes[0].axis('off')

    # Plot original image with bounding boxes and confidences
    axes[1].imshow(original_image)
    axes[1].set_title('Original Image with Bounding Boxes')
    axes[1].axis('off')

    for box, confidence in zip(boxes, confidences):
        # Plot bounding box
        rect = plt.Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], 
                             linewidth=2, edgecolor='red', facecolor='none')
        axes[1].add_patch(rect)

        # Calculate position for confidence text
        x_pos = box[2] + 5
        y_pos = box[1]

        # Plot confidence text near the bounding box
        axes[1].text(x_pos, y_pos, f'Conf: {confidence:.2f}', color='red')

    plt.tight_layout()
    plt.show()



def export_masks_images(masks_images, folder_path):
    # Create the folder if it doesn't exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Save each image in the list
    for i, img in enumerate(masks_images):
        img_path = os.path.join(folder_path, f'mask{i+1}.png')
        # Convert numpy.ndarray to PIL Image
        if isinstance(img, np.ndarray):
            # Convert numpy.ndarray to PIL Image
            img = Image.fromarray((img * 255).astype(np.uint8))
            # Save PIL Image
            img.save(img_path)
        else:
            raise ValueError(f"Unsupported image type: {type(img)}. Expected numpy.ndarray.")



def export_merged_masks_images(merged_masks_images, folder_path):
    # Create the folder if it doesn't exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    img_path = os.path.join(folder_path, f'merged.png')
    # Convert numpy.ndarray to PIL Image
    if isinstance(merged_masks_images, np.ndarray):
        # Convert numpy.ndarray to PIL Image
        merged_masks_images = Image.fromarray((merged_masks_images * 255).astype(np.uint8))
        # Save PIL Image
        merged_masks_images.save(img_path)
    else:
        raise ValueError(f"Unsupported image type: {type(merged_masks_images)}. Expected numpy.ndarray.")