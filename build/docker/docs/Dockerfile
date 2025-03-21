FROM python:3.8-slim-bullseye

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git
RUN git config --global safe.directory '*'

RUN pip install -U 'pydoc-markdown==4.8.2'
