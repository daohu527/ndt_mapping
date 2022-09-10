#!/usr/bin/env bash

set -e

REPO=daohu527/map
TAG="${REPO}:ndt-x86-18.04"

docker pull "${TAG}"

docker build -t "${TAG}" -f ndt.x86.dockerfile .

# push docker to hub.docker.com
# docker tag apollo:ndt-x86-18.04 daohu527/map:ndt-x86-18.04
# docker push daohu527/map:ndt-x86-18.04
