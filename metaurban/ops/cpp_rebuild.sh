#!/bin/bash
# 改 C++ 后:ctest + 三个 .so 手编(capi 不在 CMake!)
set -e
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sando
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd $SC/cpp && cmake --build build -j && (cd build && ctest --output-on-failure | tail -3)
g++ -O2 -shared -fPIC -std=c++17 -o capi/sando_capi.so capi/sando_capi.cpp -Iinclude -Ithird_party/eigen -Ithird_party
(cd capi && g++ -O2 -shared -fPIC -std=c++17 -o cert_capi.so cert_capi.cpp -I ../include -I ../third_party/eigen)
cd $SC/ego && g++ -O2 -shared -fPIC -std=c++17 -Wno-narrowing -w -o capi/ego_capi.so capi/ego_capi.cpp src/*.cpp -I include -I ../cpp/include -I ../cpp/third_party/eigen -I ../cpp/third_party
echo "ctest + 3 .so 全部重编完成"
