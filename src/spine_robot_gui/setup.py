from setuptools import find_packages, setup

package_name = 'spine_robot_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'PyQt5'],
    zip_safe=True,
    maintainer='Tunde Ayodeji',
    maintainer_email='aayodej3@jhu.edu',
    description='GUI to facilitate control of needle insertion robot ' \
    'for in-development semi-autonomous vertebroplasty system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = spine_robot_gui.main:main'
        ],
    },
)