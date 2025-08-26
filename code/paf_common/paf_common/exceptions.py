import traceback


def emsg_with_trace(e: Exception):
    traceback_str_list = traceback.format_exception(e)
    traceback_str = "".join(traceback_str_list)
    return f"\n{traceback_str}"
