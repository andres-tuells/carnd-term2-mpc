#! /bin/bash
apt-get update
apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets