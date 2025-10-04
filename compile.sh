#!/bin/bash



./src/can_constants/generate-files.sh -v

./after_ioc.sh

cmake -B build -G "Ninja"
cmake --build build
