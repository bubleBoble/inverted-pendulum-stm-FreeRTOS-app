#!/bin/bash

IMAGE_NAME="fedora-stm-build-base"
IMAGE_VERSION="v1.0"
CONTAINER_NAME="fedora-stm-build-base"
REMOVE_AFTER_BUILD=true

CONTAINER_RUN=(podman run
        --name ${CONTAINER_NAME}
        --rm=${REMOVE_AFTER_BUILD}
        --userns=keep-id
        --volume $(pwd):/workdir
        --workdir /workdir
        --security-opt label=disable
        --hostname ${CONTAINER_NAME})

run_debug() {
        echo "Running build in DEBUG mode"
        rm -rf ./build_cmake/debug/*
        "${CONTAINER_RUN[@]}" \
                "${IMAGE_NAME}:${IMAGE_VERSION}" \
                bash -c "make debug -j"
}

run_release() {
        echo "Running build in RELEASE mode"
        "${CONTAINER_RUN[@]}" \
                "${IMAGE_NAME}:${IMAGE_VERSION}" \
                bash -c "make release -j"
}

run_interactive() {
        echo "Running container in INTERACTIVE mode"
        "${CONTAINER_RUN[@]}" \
                --interactive \
                --tty \
                "${IMAGE_NAME}:${IMAGE_VERSION}" \
                bash
}

run_help() {
        echo "Usage:"
        echo "    -h, --help       this help"
        echo "    -d, --debug      run container to build in debug mode"
        echo "    -r, --release    run container to build in release mode"
        echo "    --it             run container in interactive mode"
}

# Parse flags
while [[ $# -gt 0 ]]; do
        case "$1" in
        -d | --debug)
                run_debug
                shift
                ;;
        -r | --release)
                run_release
                shift
                ;;
        --it)
                run_interactive
                shift
                ;;
        -h | --help)
                run_help
                shift
                ;;
        *)
                echo "Unknown option: $1"
                echo "Usage: $0 [-h|--help] [-d|--debug] [-r|--release] [--it]"
                exit 1
                ;;
        esac
done
