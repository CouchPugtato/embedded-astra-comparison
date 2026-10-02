from setuptools import find_packages, setup

package_name = 'manipulation_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simulation Maintainers',
    maintainer_email='maintainer@example.com',
    description='Policy-facing Python API for MoveIt-controlled simulated robots.',
    license='Apache-2.0',
)
