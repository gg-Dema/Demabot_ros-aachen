from setuptools import find_packages, setup
import glob


package_name = 'explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dema',
    maintainer_email='gg.dema.rm.22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = explorer.explorer:main',
        ],
    },
)
