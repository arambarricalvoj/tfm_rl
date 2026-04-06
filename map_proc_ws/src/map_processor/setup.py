from setuptools import find_packages, setup

package_name = 'map_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='javierac',
    maintainer_email='javierac@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_processor = map_processor.map_node:main',
            'laser_resync = map_processor.laser_resync:main',
            'reset_slam = map_processor.reset_slam:main',
        ],
    },
)
