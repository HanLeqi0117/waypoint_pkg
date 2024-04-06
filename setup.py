from setuptools import find_packages, setup

package_name = 'waypoint_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='k895657@kansai-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data2waypoint = gps_wp_pkg.data2waypoint:main',
            'waypoint_editor = gps_wp_pkg.waypoint_editor:main',
            'waypoint2marker = gps_wp_pkg.waypoint2marker:main'
        ],
    },
)

