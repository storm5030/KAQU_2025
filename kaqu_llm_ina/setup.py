from setuptools import find_packages, setup

package_name = 'kaqu_llm_ina'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'map.json'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kino',
    maintainer_email='realkinomail@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'STTNode = kaqu_llm_ina.STTNode:main',
            'LLMNode = kaqu_llm_ina.LLMNode:main',
            'NavigatorNode = kaqu_llm_ina.NavigatorNode:main',
        ],
    },
)
