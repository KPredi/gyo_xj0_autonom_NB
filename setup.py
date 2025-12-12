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
            'triangle_robot = gyo_xj0_autonom_NB.triangle_robot:main',
            'obstacles = gyo_xj0_autonom_NB.obstacles:main',
            
        ],
    },
)