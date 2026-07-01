from setuptools import find_packages, setup

package_name = 'stereo_pipeline'

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
    maintainer='einarj17',
    maintainer_email='ebjensen03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        #Here you register each node so ros2 run can find it
        #'node_name = package_name.file_name:main'
        'console_scripts': [
        'fake_camera = stereo_pipeline.fake_camera:main',
        'depth_node = stereo_pipeline.depth_node:main',
        'calibration_node = stereo_pipeline.calibration_node:main',
        ],
    },
)
