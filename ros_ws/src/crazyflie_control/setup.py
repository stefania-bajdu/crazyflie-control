from setuptools import find_packages, setup
import os

package_name = 'crazyflie_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'],include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',  [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.py')]),
        ('share/' + package_name + '/config', [os.path.join('config', f) for f in os.listdir('config') if f.endswith('.yaml')]),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bajdus',
    maintainer_email='stefania.bajdu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node_sequential = crazyflie_control.control_sequential:main', 
            'controller_node_parallel = crazyflie_control.control_parallel:main', 
            # 'controller_node = crazyflie_control.control_cft:main', 
            # 'cluster_node = crazyflie_control.cluster_node:main', 
        ],
    },
)
