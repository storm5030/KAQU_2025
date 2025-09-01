from setuptools import find_packages, setup

package_name = 'kaqu_hardware_interfacing'

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
    maintainer='apka',
    maintainer_email='devote010409@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bulk_read_write = kaqu_hardware_interfacing.bulk_read_write:main',
            'control_subscriber = kaqu_hardware_interfacing.control_subscriber:main',
            'present_angle_publisher = kaqu_hardware_interfacing.present_angle_publisher:main',
            'imu_data_publisher = kaqu_hardware_interfacing.imu_data_publisher:main'
        ],
    },
)
