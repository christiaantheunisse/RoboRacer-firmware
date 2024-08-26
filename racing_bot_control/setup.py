from setuptools import find_packages, setup

package_name = 'racing_bot_control'

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
    maintainer='christiaan',
    maintainer_email='christiaantheunisse@hotmail.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = racing_bot_control.keyboard_control:main'
        ],
    },
)
