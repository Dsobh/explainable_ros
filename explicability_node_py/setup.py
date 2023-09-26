import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'explicability_node_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dsobh',
    maintainer_email='dsobh@unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explicability_node = explicability_node_py.explicability_node:main',
            'explicability_node_client = explicability_node_py.explicability_node_client:main'
        ],
    },
)
