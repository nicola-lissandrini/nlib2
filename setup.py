#!/usr/bin/env python3

from setuptools import setup

package_name = 'nlib2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Nicola Lissandrini',
    author_email='',
    maintainer='',
    maintainer_email='',
    keywords=['ROS'],
    description='',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'ros_configuration = nlib2.ros_configuration:main',
        ],
    },
)