from setuptools import setup, find_packages

setup(
    name='gps-imu-integrated-navigation-system',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'keyboard',
        'djitellopy',
        'scipy',
        'matplotlib',
        'pandas',
        'geopy',
        'matplotlib',
        'pyserial',
        # Add any other dependencies here
    ],
    entry_points={
        'console_scripts': [
            'your_command_name=your_package_name.module_name:main',
            # Add any other entry points here
        ],
    },
    author='Shrey Patel',
    author_email='shreydpatel1@gmail.com',
    description='GPS-IMU Integrated Navigation System',
)
