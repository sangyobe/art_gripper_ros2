from setuptools import find_packages, setup

package_name = 'art_gripper_py'

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
    maintainer='sean',
    maintainer_email='sean.yi@hyundai.com',
    description='HMC ART Gripper tester (Python)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_client = art_gripper_py.gripper_client:main'
        ],
    },
)
