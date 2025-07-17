from setuptools import find_packages, setup

package_name = 'dnn_pkg'

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
    maintainer='jackal',
    maintainer_email='kaunghtethtun2013@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_subscriber = dnn_pkg.odom_sub:main',  # Entry point for the odometry subscriber node
            'odom_tf_subscriber = dnn_pkg.odom_tf_sub:main',  # Entry point for the odometry and TF subscriber nod
        ],
    },
)