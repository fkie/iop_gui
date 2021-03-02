from setuptools import setup

package_name = 'fkie_iop_rqt_access_control'
setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['src/' + package_name + '/handoff_dialog.ui',
                                                 'src/' + package_name + '/handoff_request.ui',
                                                 'src/' + package_name + '/iop_rqt_access_control_config.ui',
                                                 'src/' + package_name + '/iop_rqt_access_control.ui',
                                                 'src/' + package_name + '/robot.ui',
                                                 'src/' + package_name + '/system_info.ui',
                                                 'src/' + package_name + '/warning_info.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('lib/' + package_name, ['scripts/iop_rqt_access_control']),
    ],
    install_requires=['setuptools', 'fkie_iop_msgs'],
    zip_safe=True,
    author='Alexander Tiderko',
    maintainer='Alexander Tiderko',
    maintainer_email='alexander.tiderko@fkie.fraunhofer.de',
    keywords=['ROS2', 'IOP'],
    description=(
        'GUI plugin to get access control for an IOP compliant robot.'
    ),
    license='GPLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_iop_access_control = fkie_iop_rqt_access_control.main:main',
        ],
    },
)