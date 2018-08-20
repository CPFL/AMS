#!/usr/bin/env python
# coding: utf-8

import sys
import unittest
from test_arrow_helper import test_arrow_helper
from test_route_helper import test_route_helper


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    test_arrow_helper(test_loader, suite)
    test_route_helper(test_loader, suite)

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
