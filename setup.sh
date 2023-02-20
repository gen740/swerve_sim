export GAZEBO_MODEL_PATH=$(pwd)/models:$GAZEBO_MODEL_PATH

export LD_LIBRARY_PATH=$(pwd)/build:$LD_LIBRARY_PATH
export LIBRARY_PATH=$(pwd)/build:$LIBRARY_PATH
export DYLD_LIBRARY_PATH=$(pwd)/build:$DYLD_LIBRARY_PATH
export LIBPATH=$(pwd)/build:$LIBPATH
