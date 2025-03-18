# syntax = edrevo/dockerfile-plus

INCLUDE+ ./build/docker/agent/Dockerfile

RUN pip install -U sphinx sphinx-markdown-builder sphinx-autodoc-typehints myst-parser pydoc-markdown mkdocs "mkdocstrings[python]" pandoc

ADD --chmod=755 ./build/docker/docs/command.sh /command.sh
