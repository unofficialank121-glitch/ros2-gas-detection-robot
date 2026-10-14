from setuptools import find_packages, setup

package_name = 'gas_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Gas Detection and Decision Making Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = gas_robot.publisher:main',
            'subscriber = gas_robot.subscriber:main',
            'decision_maker = gas_robot.decision_maker:main',
            'gas_simulator = gas_robot.gas_simulator:main',
        ],
    },
)
