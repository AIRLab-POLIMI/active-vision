from setuptools import find_packages, setup
import os
import glob

package_name = 'av_segmentation_yolo_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "data"),
            glob.glob("data/*.png") + glob.glob("data/*.jpeg") + glob.glob("data/*.jpg"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michelelagreca',
    maintainer_email='michelelagreca.work@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yolo_world = av_segmentation_yolo_world.yolo_world:main",
            "yolo_world_test = av_segmentation_yolo_world.yolo_world_test:main",
            "yolo_world_server = av_segmentation_yolo_world.yolo_world_server:main"
        ],
    },
)
