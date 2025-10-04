#!/bin/bash

# git submodule update --init --recursive

./src/can_constants/generate-files.sh -v

./after_ioc.sh

cmake -B build -G "Ninja"
cmake --build build
