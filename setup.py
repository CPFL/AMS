#!/usr/bin/env python
# coding: utf-8

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
    version='0.3',
    description='Autoware Management System',
    long_description=long_description,
    url='https://github.com/CPFL/Autoware-Management-System',
    author='hiro-ya-iv',
    author_email='hiro.ya.iv@gmail.com',
    license=license,
    packages=find_packages(exclude=["config", "res", 'tools', 'docs', 'tests', "examples"]),
    install_requires=[
        "paho-mqtt==1.3.1",
        "transforms3d==0.3.1",
        "python-geohash==0.8.5",
        "Geohash==1.0",
        "Cerberus==1.1",
        "transitions==0.6.4"
    ],
    test_suite="tests"
)
