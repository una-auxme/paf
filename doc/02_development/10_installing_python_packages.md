# Install python packages

(Kept from previous group [paf22])

**Summary:** This page gives a short overview how to add python packages to the project.

---

## Author

Tim Dreier

## Date

7.12.2022

## Adding packages with pip

To have a unified setup every python package has to be added with a fixed version.

> Please don't install a package (inside a container) with `pip install xxx` since it would then be just installed in your specific container.

Instead, any package should be added to `code/requirements.txt`. Always set the package to a fixed version with `==` to avoid version conflicts.

An example how this file could look like is given below:

```text
torch==1.13.0
torchvision==0.1.9
```

To install the added packages run `b5 install` afterwards.

## Common Problems

Sometimes, PyCharm does not recognize installed packages on the docker container.
This leads to the problem that the program cannot be started in PyCharm via the run button, but only via command line.

A workaround for this problem is:

1. Run ```docker compose build``` in the console in the build folder.
2. Click on the python interpreter in the lower right corner and reselect it.
