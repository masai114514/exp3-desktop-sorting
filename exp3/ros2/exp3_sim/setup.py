from glob import glob
from setuptools import setup

package_name = 'exp3_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'color_detector = exp3_sim.color_detector:main',
            'truth_detector = exp3_sim.truth_detector:main',
            'pick_place_server = exp3_sim.pick_place_server:main',
            'sim_task = exp3_sim.sim_task:main',
        ],
    },
)
