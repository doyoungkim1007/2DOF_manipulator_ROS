from setuptools import setup

package_name = 'manipul_simul_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='2DOF robot manipulator package using ROS 2 and Tkinter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'joint_angles_pub_node = manipul_simul_package.joint_angles_pub_node:main',
            'set_lengths_srv_node = manipul_simul_package.set_lengths_srv_node:main',
            'parameter_server_node = manipul_simul_package.parameter_server_node:main',
            'manipulator_gui = manipul_simul_package.manipulator_gui:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manipulator_launch.py']),
    ],
)
