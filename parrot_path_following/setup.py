from setuptools import find_packages, setup

package_name = 'parrot_path_following'

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
    maintainer='amir',
    maintainer_email='asgharnia4@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "follow_path = parrot_path_following.follow_defined_trajectory:main",
            "emergency_land = parrot_path_following.emergency_land_code:main"
        ],
    },
)
