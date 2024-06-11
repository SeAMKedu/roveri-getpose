from setuptools import find_packages, setup

package_name = 'getpose'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hannu Hakalahti',
    maintainer_email='hannu.hakalahti@seamk.fi',
    description='Collect the pose of the TurtleBot 4',
    license='CC-BY-4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = getpose.main:main'
        ],
    },
)
