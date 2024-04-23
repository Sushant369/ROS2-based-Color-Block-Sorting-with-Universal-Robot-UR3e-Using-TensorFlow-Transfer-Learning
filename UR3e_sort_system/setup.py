from setuptools import find_packages, setup

package_name = 'mini_project'

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
    maintainer='sushant',
    maintainer='case',
    maintainer_email='sushantshelar121@gmail.com',
    maintainer_email='caseashton2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'robot2_handler_test = UR3e_sort_system.robot2_handler_test :main',
        'test_block = UR3e_sort_system.test_block:main',
        'robot2_handler = UR3e_sort_system.robot2_handler :main',
        ],
    },
)
