Sure, I can guide you through the steps to install `depthai-core` on Ubuntu 18.04 from source.

Before you start, you will need to have some prerequisites installed on your system:

- CMake (version 3.2 or later)
- Python (version 3.6 or later)
- Git

Let's start by installing these prerequisites:

```bash
sudo apt update
sudo apt install cmake python3 git
```

Once you have these prerequisites installed, you can proceed with the following steps to install `depthai-core`:

1. **Clone the repository.**

First, you should clone the `depthai-core` repository from GitHub:

```bash
git clone https://github.com/luxonis/depthai-core.git
```

2. **Navigate to the cloned directory.**

Once the repository is cloned, navigate to the `depthai-core` directory:

```bash
cd depthai-core
git submodule update --init --recursive
```

3. **Create a build directory.**

It is a good practice to create a separate directory for the build files:

```bash
mkdir build && cd build
```

4. **Configure the build.**

Next, you need to configure the build process using CMake. If you want to build the examples, you should pass `-DBUILD_EXAMPLES=ON` to the CMake command:

```bash
cmake ..
```

Or, if you want to build examples:

```bash
cmake -DBUILD_EXAMPLES=ON ..
```

5. **Build the project.**

After configuration, you can build the project:

```bash
make -j1
```

The `-j1` option tells `make` to use as many cores as possible to speed up the build process.

After these steps, `depthai-core` should be built and ready to use. Note that if you want to use the library in your own project, you should include the `depthai/depthai.hpp` header and link against the `depthai-core` library.

```bash
make install
```

Reference:
- [DepthAI Core GitHub Repository](https://github.com/luxonis/depthai-core)

```bash
ls /home/orca/depthai-core/build
export CMAKE_PREFIX_PATH=/home/orca/depthai-core/build:${CMAKE_PREFIX_PATH}
export depthai_DIR=/home/orca/depthai-core/build

sudo apt-get install libbz2-dev


source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh
export OPENBLAS_CORETYPE=ARMV8

export CMAKE_PREFIX_PATH=/home/orca/depthai-core/build:${CMAKE_PREFIX_PATH}
export depthai_DIR=/home/orca/depthai-core/build
```