#!/bin/bash

# Make sure that this script runs from its own location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "running from $SCRIPT_DIR"

IMAGE_NAME="fedora-stm-build-base"
IMAGE_VERSION="v1.0"

CONTAINER_FILE="$(pwd)/Containerfile"
UID=$(id -u)
GID=$(id -g)
USER=$(id -un)
if [[ "$OS" == "Windows_NT" ]]; then
    GROUP=$(id -un)
else
    GROUP=$(id -gn)
fi

podman build \
        --tag "${IMAGE_NAME}:${IMAGE_VERSION}" \
        --file="${CONTAINER_FILE}" \
        --build-arg UID=${UID} \
        --build-arg GID=${GID} \
        --build-arg USERNAME=${USER} \
        --build-arg GROUPNAME=${GROUP} \
        .
