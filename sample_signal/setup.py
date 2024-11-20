from setuptools import find_packages, setup 
import os 
from glob import glob

package_name = 'sample_signal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitry',
    maintainer_email='dmalad@rptu.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_signal = sample_signal.sample_signal:main',
            'sine = sample_signal.sine_wave:main',
            'imu = sample_signal.imu:main',
        ],
    },
)
