from setuptools import setup

setup(
    name='LeArmKinematics',
    version='0.1',
    description='Python packages for 6Dof LeArm',
    packages=['learm'],
    install_requires=[
        'numpy',
        'matplotlib'
    ],
)