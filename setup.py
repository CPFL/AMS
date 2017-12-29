#!/usr/bin/env python
# coding: utf-8


# source ~/workspace/Autoware/ros/devel/setup.bash
# curl -L https://raw.githubusercontent.com/pyenv/pyenv-installer/master/bin/pyenv-installer | bash
# echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> ~/.bashrc
# echo 'eval "$(pyenv init -)"' >> ~/.bashrc
# echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
# source ~/.bashrc
# pyenv -v
#
# pyenv install 3.6.2
# pyenv local 3.6.2


from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

with open(path.join(here, 'LICENSE'), encoding='utf-8') as f:
    license = f.read()

setup(
    name='ams',
    version='0.1',
    description='Autoware Management System',
    long_description=long_description,
    url='https://github.com/CPFL/Autoware-Management-System',
    author='hiro-ya-iv',
    author_email='hiro.ya.iv@gmail.com',
    license=license,
    packages=find_packages(exclude=["config", "res", 'tools', 'docs', 'tests', "examples"]),
    install_requires=[
        "paho-mqtt==1.3.1",
        "python-dotenv==0.7.1",
        "transforms3d==0.3.1",
        "numpy==1.13.3",
        "flask==0.12.2",
        "flask-cors==3.0.3",
        "geohash==1.0",
    ],
    test_suite="tests"
)