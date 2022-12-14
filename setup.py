from setuptools import find_packages
from setuptools import setup


package_name = 'create3_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilarioazzollini',
    maintainer_email='ilario.azzollini92@gmail.com',
    description='Motion control algorithms for Create 3',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_server = create3_control.controller_server:main'
        ],
    },
)
