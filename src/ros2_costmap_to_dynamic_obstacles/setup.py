from setuptools import find_packages, setup

package_name = 'ros2_costmap_to_dynamic_obstacles'

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
    maintainer='daopctn',
    maintainer_email='daopctn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "costmap_converter_node = ros2_costmap_to_dynamic_obstacles.costmap_converter_node:main"
        ],
    },
)
