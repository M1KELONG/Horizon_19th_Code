from setuptools import setup

package_name = 'qrcoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qrcoder = qrcoder.qrcoder:main',
            'camera_dis = qrcoder.camera_dis:main',
            'rosbridge_node = qrcoder.rosbridge_node:main',
            'racing_control = qrcoder.racing_control:main',
            'start_sign = qrcoder.start_sign:main',
        ],
    },
)
