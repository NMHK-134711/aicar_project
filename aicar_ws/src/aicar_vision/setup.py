from setuptools import find_packages, setup
import os           # <-- os 임포트
from glob import glob # <-- glob 임포트

package_name = 'aicar_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'calibration_data'), 
         glob(os.path.join('calibration_data', '*.p'))),
    ],
    install_requires=['setuptools', 'numpy'], # numpy가 이미 포함되어 있음
    zip_safe=True,
    maintainer='rapie',
    maintainer_email='rapie@todo.todo',
    description='Advanced lane detection and BEV node for AI car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector_node = aicar_vision.lane_detector_node:main',
            'fake_detector_node = aicar_vision.fake_detector_node:main',
        ],
    },
)