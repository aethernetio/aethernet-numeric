# aethernet-numeric

## Overview

TODO: Overview

## Build tests

Clone the repository:

```sh
git clone https://github.com/aethernetio/aethernet-numeric.git
```

Cd into the repository and update submodules:

```sh
cd aethernet-numeric
git submodule update --init --recursive
```

Make build directory:

```sh
mkdir build
cd build
```

Use CMake to generate build files.
Add the `-DAE_BUILD_TESTS=On` flag to enable unit tests.

```sh
cmake -DAE_BUILD_TESTS=On ..
```

Build the project and run tests:

```sh
cmake --build .
ctest .
```
