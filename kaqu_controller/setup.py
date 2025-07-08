from setuptools import find_packages, setup

package_name = 'kaqu_controller'

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
            'RobotManagerNode = kaqu_controller.KaquCmdManager.RobotManagerNode:main',
            'QuadrupedControllerNode = kaqu_controller.KaquCmdManager.AnglePublisher:main',
            'debug = kaqu_controller.KaquCmdManager.test:main'
            
        ],
    },
)