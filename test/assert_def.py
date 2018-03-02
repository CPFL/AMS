#!/usr/bin/env python
# coding: utf-8

import unittest
import json
import numpy as np
from numpy import array as np_array


def assert_np_array(first, second, msg=None):
    """Assert that two multi-line strings are equal."""

    if first != second:
        # don't use difflib if the strings are too long
        if (len(first) > self._diffThreshold or len(second) > self._diffThreshold):
            self._baseAssertEqual(first, second, msg)
        firstlines = first.splitlines(True)
        secondlines = second.splitlines(True)
        if len(firstlines) == 1 and first.strip('\r\n') == first:
            firstlines = [first + '\n']
            secondlines = [second + '\n']
        standardMsg = '%s != %s' % (safe_repr(first, True), safe_repr(second, True))
        diff = '\n' + ''.join(difflib.ndiff(firstlines, secondlines))
        standardMsg = self._truncateMessage(standardMsg, diff)
        self.fail(self._formatMessage(msg, standardMsg))



class unittest_AMS(unittest):

    def __init__(self):
        super.__init__()


    def assert_np_array(first, second, msg=None):

        """Assert that two multi-line strings are equal."""
        self.assertIsInstance(first, basestring,
                              'First argument is not a string')
        self.assertIsInstance(second, basestring,
                          'Second argument is not a string')

        if first != second:
            # don't use difflib if the strings are too long
            if (len(first) > self._diffThreshold or len(second) > self._diffThreshold):
            self._baseAssertEqual(first, second, msg)
            firstlines = first.splitlines(True)
            secondlines = second.splitlines(True)
            if len(firstlines) == 1 and first.strip('\r\n') == first:
                firstlines = [first + '\n']
                secondlines = [second + '\n']
            standardMsg = '%s != %s' % (safe_repr(first, True), safe_repr(second, True))
            diff = '\n' + ''.join(difflib.ndiff(firstlines, secondlines))
            standardMsg = self._truncateMessage(standardMsg, diff)
            self.fail(self._formatMessage(msg, standardMsg))
