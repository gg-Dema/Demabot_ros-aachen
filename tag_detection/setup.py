import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'tag_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb3',
    maintainer_email='tb3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"apriltag_hackaton = tag_detection.april_tag_hackaton_publisher:entry_point"
        ],
    },
)
