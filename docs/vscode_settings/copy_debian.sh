#!/bin/bash
cd "$(dirname "$0")"
mkdir ../../.vscode
cp ./DEBIAN_c_cpp_properties.json ../../.vscode/c_cpp_properties.json
