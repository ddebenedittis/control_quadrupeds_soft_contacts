from setuptools import setup

package_name = 'terrain_estimator'

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
    maintainer='Davide De Benedittis',
    maintainer_email='davide.debenedittis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_estimator_node = terrain_estimator.terrain_estimator_node:main',
        ],
    },
)
