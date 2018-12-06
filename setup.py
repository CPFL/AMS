#!/usr/bin/env python
# coding: utf-8

from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, "./ams/_version.py")) as f:
    version = f.read().split("\"")[-2]

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

with open(path.join(here, 'LICENSE'), encoding='utf-8') as f:
    _license = f.read()

setup(
    name='ams',
    version=version,
    description='Autoware Management System',
    long_description=long_description,
    url='https://github.com/CPFL/AMS',
    author='hiro-ya-iv',
    author_email='hiro.ya.iv@gmail.com',
    license=_license,
    packages=find_packages(exclude=["config", "res", 'tools', 'docs', 'tests', "examples"]),
    install_requires=[
        "python-geohash==0.8.5",
        "Cerberus==1.1",
        "PyYAML==3.13",
        "polyline==1.3.2"
    ],
    test_suite="tests"
)
