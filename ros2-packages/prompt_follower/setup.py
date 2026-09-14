import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'prompt_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='Denis Molnar Ardelean',
    maintainer_email='denismolnar6704@gmail.com',
    description='Prompt-based navigation using YOLOE',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'prompt_follower = prompt_follower.prompt_follower:main',
        ],
    },
)
