from typing import Optional, Any
import py_trees


class Blackboard(py_trees.blackboard.Blackboard):
    """py_trees.blackboard.Blackboard with added functionality

    New methods:
    - try_get: returns None if a variable does not exist in the blackboard.
      (Instead of raising an exception)
    """

    def try_get(self, variable_name: str) -> Optional[Any]:
        """Get a variable from the blackboard.

        Compared to the get() method, this method does not raise an exception
        when variable_name does not exist, but returns None instead.

        Args:
            variable_name (str): of the variable to get,
              can be nested, e.g. battery.percentage

        Returns:
            Optional[Any]:
            - None, if the variable_name does not exist
            - Any value, else
        """
        if self.exists(variable_name):
            return self.get(variable_name)
        return None
