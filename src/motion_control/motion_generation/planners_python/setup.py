from setuptools import setup

package_name = 'planners_python'
planners = 'planners_python/planners'
utils = 'planners_python/utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, planners, utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davide',
    maintainer_email='davide.debenedittis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_mjp_node = planners_python.planner_mjp_node:main',
            'planner_static_walk_node = planners_python.planner_static_walk_node:main',
        ],
    },
)
