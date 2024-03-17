from setuptools import setup

setup(
    name='LeArmKinematics',
    version='0.1',
    description='Python packages for 6Dof LeArm',
    packages=['learm'],
    install_requires=[
        'numpy>=1.26.2',
        'matplotlib>=3.8.2'
    ],
)