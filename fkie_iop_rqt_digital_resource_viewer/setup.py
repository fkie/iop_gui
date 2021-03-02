from setuptools import setup

package_name = 'fkie_iop_rqt_digital_resource_viewer'
setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['src/' + package_name + '/iop_rqt_digital_resource_viewer.ui',
                                                 'src/' + package_name + '/iop_rqt_digital_resource_viewer_config.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('lib/' + package_name, ['scripts/iop_rqt_digital_resource_viewer']),
    ],
    install_requires=['setuptools', 'fkie_iop_msgs', 'fkie_iop_ocu_slavelib'],
    zip_safe=True,
    author='Alexander Tiderko',
    maintainer='Alexander Tiderko',
    maintainer_email='alexander.tiderko@fkie.fraunhofer.de',
    keywords=['ROS2', 'IOP'],
    description=(
        'Shows the RTSP stream discovered for IOP complient robot.'
    ),
    license='GPLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_iop_digital_resource_viewer = iop_rqt_digital_resource_viewer.main:main',
        ],
    },
)