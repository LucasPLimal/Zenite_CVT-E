import os
from glob import glob

from setuptools import setup

package_name = 'bluetooth_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danilo',
    maintainer_email='voce@exemplo.com',
    description='Gerenciador de Bluetooth dos robôs (procura, pareamento, conexão RFCOMM e transmissão)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bluetooth_node = bluetooth_node.node:main',
        ],
    },
)
