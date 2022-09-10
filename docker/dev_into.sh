#!/usr/bin/env bash

REPO=apollo
TAG="${REPO}:ndt-x86-18.04"

docker run -it -v $(pwd):/home/ndt_mapping --net=host "${TAG}" /bin/bash
