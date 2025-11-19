from setuptools import find_packages, setup

package_name = 'canalchecker_dev'

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
    maintainer='yb',
    maintainer_email='yb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'FollowActionServer = canalchecker_dev.ActionServer.FollowActionServer:main',
            'AlignActionServer = canalchecker_dev.ActionServer.AlignActionServer:main',
            'DriveActionServer = canalchecker_dev.ActionServer.DriveActionServer:main',
            'ActionServerHandler = canalchecker_dev.ActionServerHandler:main'
        ],
    },
)
