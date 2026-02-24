from setuptools import setup

package_name = 'geodesic_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['launch/' + package_name + '_launch.py']),
        ('share/' + package_name + '/config', ['config/alignment_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WilliamLX',
    maintainer_email='william@example.com',
    description='3D perception and point cloud registration for ProjectGeodesic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_alignment_node = geodesic_perception.global_alignment_node:main',
            'teaching_gui = geodesic_perception.teaching_gui:main',
            'visualize_pointcloud = geodesic_perception.visualize_pointcloud:main',
        ],
    },
)
