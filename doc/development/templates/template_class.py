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
import os
import sys

# Use from x import y where x is the package prefix and y is the module name with no prefix.
from matplotlib import pyplot

# Use import y as z only when z is a standard abbreviation (e.g., np for numpy).
import numpy as np


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
        # use a trailing underscore to avoid collision of attribute names with reserved keywords
        self.if_ = False

    # function names should be lower case with underscores to improve readability
    # always use self as first argument for istance methods
    def test_function1(self):
        pass

    # single blank line between method definitions
    # always use cls as first argument for class functions
    def test_function2(cls):
        pass

    #############################
    #   5. Comments             #
    #############################

    def test_function3(self):  # inline comment
        # This is a block comment
        # It goes over multiple lines
        # All comments start with a blank space
        pass

    #############################
    #   6. Docstrings           #
    #############################
    def test_function4(self, param1, param2):
        # This docstring style is the default google style of the autoDocstring extension and helps with automated API documentation creation
        """This is the description of the function.

        Args:
            param1 (_type_): _description_
            param2 (_type_): _description_
        """
        pass

    # main function of the class
    def main(self):
        print("Hello World")


# main function, to be executed when the python file is executed
if __name__ == "__main__":
    runner = TestClass()
    runner.main()
