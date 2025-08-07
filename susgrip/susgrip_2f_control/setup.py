from setuptools import setup
import os
from glob import glob

package_name = 'susgrip_2f_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongxiem',
    maintainer_email='nghia.nguyenkhac@apicoorobotic.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'with_cmd = susgrip_2f_control.with_cmd:main',
            'with_gui = susgrip_2f_control.with_gui:main',
        ],
    },
)
