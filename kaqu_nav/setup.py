from setuptools import find_packages, setup

package_name = 'kaqu_nav'

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
    maintainer='dongryun',
    maintainer_email='storm5030@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_path_server = kaqu_nav.follow_path_server:main',
            'follow_path_client = kaqu_nav.follow_path_client:main',
        ],
    },
)
