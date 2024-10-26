# rm -r build

clear

# compile and run
mkdir -p build

cd ./build

cmake .. &&
make -j7 &&
./qt_pclview