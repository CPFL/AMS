#!/usr/bin/env python
# coding: utf-8

from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, '../../README.md'), encoding='utf-8') as f:
    long_description = f.read()

with open(path.join(here, '../../LICENSE'), encoding='utf-8') as f:
    license = f.read()

setup(
    name='ams',
    version='0.2',
    description='Autoware Management System',
    long_description=long_description,
    url='https://github.com/CPFL/Autoware-Management-System',
    author='hiro-ya-iv',
    author_email='hiro.ya.iv@gmail.com',
    license=license,
    packages=["ams"],
    install_requires=[
        "paho-mqtt==1.3.1",
        "python-dotenv==0.7.1",
        "transforms3d==0.3.1",
        "geohash==1.0",
        "python-geohash==0.8.5",
    ],
    test_suite="tests"
)
