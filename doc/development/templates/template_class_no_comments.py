#!/usr/bin/env python

# import os
# import sys
# from matplotlib import pyplot
# import numpy as np


class TestClass:

    MAX_VELOCITY = 40.0

    def __init__(self):
        self.x = 0.0
        self._name = "Max"
        self.if_ = False

    def test_function(self, param1: int, param2: float) -> str:
        """_summary_

        Args:
            param1 (int): _description_
            param2 (float): _description_

        Returns:
            str: _description_
        """
        pass

    def main(self):
        """_summary_"""
        print("Hello World")


if __name__ == "__main__":
    runner = TestClass()
    runner.main()
