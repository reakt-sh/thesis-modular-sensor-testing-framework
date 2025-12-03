from setuptools import find_packages, setup
import os

package_name = 'rail_demo'

def collect(subdir: str):
    """
    Collect all files under rail_demo/<subdir> and create data_files entries
    that preserve the relative directory structure.

    Example:
      rail_demo/models/gazebo_train/model.sdf
    will be installed to:
      share/rail_demo/models/gazebo_train/model.sdf
    """
    data = []
    base = os.path.join(package_name, subdir)
    for root, _, names in os.walk(base):
        if not names:
            continue
        # relative directory inside the package, e.g. "models/gazebo_train"
        rel_dir = os.path.relpath(root, package_name)
        # install directory under share/, e.g. "share/rail_demo/models/gazebo_train"
        install_dir = os.path.join('share', package_name, rel_dir)
        src_files = [os.path.join(root, n) for n in names]
        data.append((install_dir, src_files))
    return data

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