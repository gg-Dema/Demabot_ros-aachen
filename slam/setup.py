import os
from setuptools import find_packages, setup
from glob import glob
package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob('launch/*.yaml')),
        (os.path.join("share", package_name, "maps"), (glob("maps/*"))),
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
        ],
    },
)
