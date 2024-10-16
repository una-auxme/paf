# Sources:
# https://peps.python.org/pep-0008/#class-names
# https://pep8.org/#function-and-method-arguments

#############################
#   1. Shebang-Line         #
#############################
# needed only for files that are intended to be executed

# !/usr/bin/env python


#############################
#   2. Imports              #
#############################
# imports are always on the top of a file

# imports in seperate lines
# import os
# import sys

# Use from x import y where x is the package prefix and
# y is the module name with no prefix.
# from matplotlib import pyplot

# Use import y as z only when z is a standard abbreviation (e.g., np for numpy).
# import numpy as np


# two  blank lines between top level functions and class definition


#############################
#   3. Class-Defintion      #
#############################
# class names should be capitalized words
class TestClass:
    #############################
    #   4. Method-Definition    #
    #############################
    # constants should be upper case with underscores to improve readability
    MAX_VELOCITY = 40.0

    # the __init__.py constructor should always be the first class method
    def __init__(self):
        self.x = 0.0
        # one leading underscore for non-public instance and method names
        self._name = "Max"
        # use a trailing underscore to avoid collision of attribute names
        # with reserved keywords
        self.if_ = False

    # This docstring style is the default google style of the autoDocstring
    # extension for automated API documentation creation
    def test_function(self, param1: int, param2: float) -> str:
        """_summary_

        Args:
            param1 (int): _description_
            param2 (float): _description_

        Returns:
            str: _description_
        """
        pass

    # main function of the class
    def main(self):
        print("Hello World")


# main function, to be executed when the python file is executed
if __name__ == "__main__":
    runner = TestClass()
    runner.main()
