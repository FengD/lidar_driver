# Share

  > This folder containes all the share modules.

## Compile

 * For x86_64 pc
  > Create a folder named `build`
  > Go into the folder `build`
  > `cmake -DCMAKE_BUILD_TYPE=Release ..` generate Makefile
  > `make` and `make install`
  > the libs will be generated in each `build` folder of each lib.

 * For Arm compile
  > (Optional) Edit the `toolchain_*.cmake` file, change the path of the `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER`, and the path of target enviroment
  > Create a folder named `build`
  > Go into the folder `build`
  > Generate Makefile
  >> * For px2: `cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain_px2.cmake _DCMAKE_BUILD_TYPE=Release ..`
  >> * For s32v: `cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain_s32v.cmake _DCMAKE_BUILD_TYPE=Release ..`
  > `make` and `make install`
  > the libs will be generated in each `build` folder of each lib.
