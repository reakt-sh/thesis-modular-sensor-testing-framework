from setuptools import find_packages, setup
import os

package_name = 'rail_demo'

def collect(subdir: str):
    files = []
    base = os.path.join(package_name, subdir)
    for root, _, names in os.walk(base):
        for n in names:
            files.append(os.path.join(root, n))
    return [('share/{}/{}'.format(package_name, subdir), files)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + collect('worlds') + collect('models') + collect('config') + collect('launch'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Horn',
    maintainer_email='david@example.com',
    description='Train demo world & tools',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # 'controller = rail_demo.controller:main',
        ],
    },
)