from setuptools import find_packages, setup

package_name = 'speech_client'

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
    maintainer='Denis Molnar Ardelean',
    maintainer_email='denismolnar6704@gmail.com',
    description='Diction + translation recognition node using Whisper.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "speech_client = speech_client.speech_client:main",
        ],
    },
)
