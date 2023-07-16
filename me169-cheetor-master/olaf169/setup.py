from setuptools import setup
from glob import glob

package_name = 'olaf169'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/maps',    glob('maps/*')),
        ('share/' + package_name + '/csv', glob('csv/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder  = olaf169.encoder:main',
            'wheelcontrol = olaf169.wheelcontrol:main',
            'driver = olaf169.driver:main',
            'odometry = olaf169.odometry:main',
            'gyro = olaf169.gyro:main',
            'drive_auto = olaf169.drive_auto:main',
            'lidar = olaf169.rplidarfix:main',
            'localize = olaf169.localize:main',
            'testnode = olaf169.testnode:main',
            'planner = olaf169.planner:main',
            'pose_toggle = olaf169.pose_toggle:main',
            'mapping = olaf169.mapping:main',
            'sentry_move = olaf169.sentry_move:main',
            'global_explore = olaf169.global_explore:main',
            'sentry_drive = olaf169.sentry_drive:main',
            'check_complete = olaf169.check_complete:main', 
            'temporal_prm = olaf169.temporal_prm:main', 
            'temporal_planner = olaf169.temporal_planner:main',
        ],
    },
)
