from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'susgrip_2f_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'susgrip_2f_driver'), glob('susgrip_2f_driver/*.py')),
        (os.path.join('share', package_name, 'susgrip_2f_utility'), glob('susgrip_2f_utility/*.py')),
        (os.path.join("lib", package_name), ["scripts/susgrip_2f_gui.py"]),  # Install script
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongxiem',
    maintainer_email='nghia.nguyenkhac@apicoorobotic.com',
    description='Apicoo Robotics SusGrip 2F Gripper ROS2 package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'susgrip_2f_driver = susgrip_2f_driver.susgrip_2f_driver:main'
        ],
    },
)
