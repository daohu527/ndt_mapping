#!/usr/bin/env bash

REPO=daohu527/map
TAG="${REPO}:ndt-x86-18.04"

docker pull "${TAG}"

docker run -it -v $(pwd):/home/ndt_mapping --net=host "${TAG}" /bin/bash
