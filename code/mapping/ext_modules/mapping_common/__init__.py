import cython

if cython.compiled:
    print("mapping_common is compiled!")
else:
    print("mapping_common is not compiled!")
