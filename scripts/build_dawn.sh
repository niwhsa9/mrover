#!/bin/bash

git submodule update --init deps/dawn

DAWN_USE_WAYLAND=$( [ ! -z "${WAYLAND_DISPLAY}" ] && echo "ON" || echo "OFF" )

pushd deps/dawn || exit
CC=clang CXX=clang++ cmake \
    -B out/Release \
    -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_CXX_FLAGS="-march=native" \
    -D DAWN_FETCH_DEPENDENCIES=ON \
    -D DAWN_ENABLE_DESKTOP_GL=OFF \
    -D DAWN_ENABLE_OPENGLES=OFF \
    -D DAWN_ENABLE_NULL=OFF \
    -D DAWN_BUILD_SAMPLES=OFF \
    -D TINT_BUILD_DOCS=OFF \
    -D TINT_BUILD_TESTS=OFF \
    -D DAWN_ENABLE_INSTALL=ON \
    -D BUILD_SHARED_LIBS=ON \
    -D DAWN_USE_GLFW=OFF \
    -D TINT_BUILD_CMD_TOOLS=OFF \
    -D TINT_BUILD_DOCS=OFF \
    -D TINT_BUILD_TESTS=OFF \
	-D DAWN_USE_WAYLAND="$DAWN_USE_WAYLAND"
cmake --build out/Release
