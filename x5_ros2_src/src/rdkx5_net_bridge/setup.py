from setuptools import setup

package_name = 'rdkx5_net_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/net_bridge.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='BLE/Wi‑Fi topic ingress bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'net_bridge = rdkx5_net_bridge.net_bridge:main',
        ],
    },
)