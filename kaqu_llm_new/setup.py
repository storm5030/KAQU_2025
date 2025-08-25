from setuptools import find_packages, setup

package_name = 'kaqu_llm_new'

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
    maintainer='woojin',
    maintainer_email='kimwoojin.alex@gmail.com',
    description='LLM tools for KAQU',
    license='Apache-2.0',
    tests_require=['pytest'],

    include_package_data=True,
    package_data={
        'kaqu_llm_new': ['data/*.json']
    },

    entry_points={
        'console_scripts': [
            'stt_node = kaqu_llm_new.stt:main',
            'navigate_node = kaqu_llm_new.navigate:main',
            'llm_node = kaqu_llm_new.llm:main',

        ],
    },
)
