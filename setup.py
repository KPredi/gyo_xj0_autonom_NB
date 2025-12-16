from setuptools import setup

package_name = 'gyo_xj0_autonom_NB'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/indito.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdfghj',
    maintainer_email='dfg',
    description='dfgh',
    license='GNU General Public License v3.0',
    entry_points={
        'console_scripts': [
            'robot = gyo_xj0_autonom_NB.robot:main',
            'akadalyok = gyo_xj0_autonom_NB.akadalyok:main',
            'utvonal = gyo_xj0_autonom_NB.utvonal:main',
            'lidar = gyo_xj0_autonom_NB.lidar:main',
        ],
    },
)