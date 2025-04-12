from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup, find_packages

setup(
    name='robogardener',
    version='0.0.0',
    packages=['robogardener'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robogardener']),
        ('share/robogardener', ['package.xml']),
#ADDED
        (os.path.join('share', 'robogardener', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'robogardener', 'description'), glob('description/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noiringraham',
    maintainer_email="grahanoi@students.zhaw.ch",
    description='Robogardener Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = robogardener.nodes.state_publisher:main',
            # Weitere Python-Nodes hier eintragen
        ],
    },
)