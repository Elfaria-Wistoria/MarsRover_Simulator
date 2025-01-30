# setup.py
from setuptools import setup, find_packages

setup(
    name="mars-rover-simulator",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        'numpy>=1.21.0',
        'matplotlib>=3.4.0',
        'PyQt5>=5.15.0',
        'pandas>=1.3.0',
        'scipy>=1.7.0',
        'pytest>=6.2.0',
    ],
)