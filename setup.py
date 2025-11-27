from setuptools import setup, find_packages

setup(
    name='elena_example',
    version='0.1.0',
    description='A package for CRX description and robot simulation.',
    author='swilcock0',
    url='https://github.com/swilcock0/elena_example',
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        'numpy',
        'matplotlib',
        'pybullet'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)