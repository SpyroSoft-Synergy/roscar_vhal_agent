from setuptools import find_packages, setup

package_name = 'py_ros2_android_vhal_srv'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sławomir Cielepak',
    maintainer_email='sie@spyro-soft.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vhal_agent = py_ros2_android_vhal_srv.vhal_agent:main',
            'hvac_client = py_ros2_android_vhal_srv.hvac_test_client:main.',
        ],
    },
)
