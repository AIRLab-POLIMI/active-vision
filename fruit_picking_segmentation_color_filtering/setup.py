from setuptools import find_packages, setup
import os
import glob

package_name = 'fruit_picking_segmentation_color_filtering'

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
            "color_filter = fruit_picking_segmentation_color_filtering.color_filter:main",
        ],
    },
)
