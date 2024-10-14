#!/usr/bin/env python

import os
import sys
from matplotlib import pyplot
import numpy as np


class TestClass:

    MAX_VELOCITY = 40.0

    def __init__(self):
        self.x = 0.0
        self._name = "Max"
        self.if_ = False

    def test_function1(self, param1):
        pass

    def test_function2(cls):
        """
        
        :return:
        """
        pass

    def test_function3(self):       # inline comment
        # This is a block comment
        # It goes over multiple lines
        # All comments start with a blank space
        pass

    def test_function4(self, param1, param2):
        """
        This is the description of the function.

        :param param1: first parameter
        :param param2: second parameter
        :return: return value(s)
        """
        pass

    def test_function5(self, param1, param2):
        """_summary_

        :param param1: _description_
        :param param2: _description_
        :return: _description_
        """
        return param1

    def main(self):
        """_summary_
        """
        print("Hello World")

if __name__ == "__main__":
    runner = TestClass()
    runner.main()
