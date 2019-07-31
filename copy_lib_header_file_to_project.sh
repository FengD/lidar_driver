#!/bin/bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make install

project_path=$1
# you can commit the module no need
lidar_driver="lidar"
declare -A modules=(
                    ["lidar_driver"]="${lidar_driver}"
                    )

if [ ! -n "$project_path" ] ;
then
  echo "project path not given"
else
  echo "project path is ${project_path}"
  for key in ${!modules[@]}
    do
      if [ -n "${modules[$key]}" ] ;then
        cp ./libs/$key/build/include/* ${project_path}/src/${modules[$key]}/include/$key/
        cp ./libs/$key/build/lib/* ${project_path}/src/${modules[$key]}/lib/
        echo "copy $key finish"
        echo ""
      fi
    done
fi
