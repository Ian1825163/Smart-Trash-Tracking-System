from setuptools import find_packages, setup

package_name = 'trash_script'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['best.pt']),
        ('share/' + package_name, ['yolov8n.pt']),
        ('share/' + package_name, ['best2.pt']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi5',
    maintainer_email='rpi5@todo.todo',
    description='ROS 2 nodes for YOLO-based trash tracking and mecanum-wheel motion control.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vision = trash_script.vision:main',
            'new_vision = trash_script.new_vision:main',
            'trajectory = trash_script.trajectory:main',
            'move = trash_script.move:main',
        ],
    },
)
