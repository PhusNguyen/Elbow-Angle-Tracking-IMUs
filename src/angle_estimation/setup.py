from setuptools import find_packages, setup

package_name = 'angle_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/angle_estimation.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Phu',
    maintainer_email='nguyen.p3@northeastern.edu',
    description='Angle Estimation Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'relative_quat_method = angle_estimation.relative_quat_method:main',
            'vector_method = angle_estimation.vector_method:main',
        ],
    },
)
