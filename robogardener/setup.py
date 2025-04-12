from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'robogardener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'state_publisher'  # Wenn die Datei direkt im nodes-Ordner liegt
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
#ADDED
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
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
            'state_publisher = state_publisher:main',
        ],
    },
)