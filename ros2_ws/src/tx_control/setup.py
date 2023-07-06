from setuptools import setup

package_name = 'tx_control'

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
    maintainer='minarady',
    maintainer_email='minarady@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
            'talker = tx_control.publisher_member_function:main',
            'listener = tx_control.subscriber_member_function:main',
            ],
        },
)
