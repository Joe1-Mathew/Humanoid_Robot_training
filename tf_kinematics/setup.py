from setuptools import find_packages, setup

package_name = 'tf_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TF, forward kinematics, and inverse kinematics exercises',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fk_check_node = tf_kinematics.fk_check_node:main',
            'target_slider_gui = tf_kinematics.target_slider_gui:main',
            'analytical_ik = tf_kinematics.analytical_ik:main',
        ],
    },
)