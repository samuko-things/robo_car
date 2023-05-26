from setuptools import setup

package_name = 'pynput_ackermann_teleop_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'pynput_ackermann_teleop_drive = pynput_ackermann_teleop_drive.pynput_ackermann_teleop_drive:main',
        ],
    },
)
