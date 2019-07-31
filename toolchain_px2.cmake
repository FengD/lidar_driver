# this one is important
SET(CMAKE_SYSTEM_NAME Linux)
# this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER "/home/ding/NXP/S32DS_Vision_v2018.R1/Cross_Tools/gcc-6.3-arm64-linux/bin/aarch64-linux-gnu-gcc")
SET(CMAKE_CXX_COMPILER "/home/ding/NXP/S32DS_Vision_v2018.R1/Cross_Tools/gcc-6.3-arm64-linux/bin/aarch64-linux-gnu-g++")

# cross compile arm
SET(ARM_COMPILE TRUE)

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH "/d/s32vtoolchians/DMS_HWV1_SW180/sysroots/aarch64-fsl-linux")

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
