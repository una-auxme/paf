"""Helpers for formatting exceptions for logs and status messages."""

import traceback


def emsg_with_trace(e: Exception) -> str:
    """Return a formatted traceback string for the given exception."""
    traceback_str_list = traceback.format_exception(e)
    traceback_str = "".join(traceback_str_list)
    return f"\n{traceback_str}"
