#!/usr/bin/env python
# coding: utf-8

import sys
import unittest
from test_arrow import test_arrow
from test_route import test_route


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    test_arrow(test_loader, suite)
    test_route(test_loader, suite)

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
