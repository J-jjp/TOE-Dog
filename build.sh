sudo apt-get install cmake xorg-dev libglu1-mesa-dev


sudo mkdir glfw/build
sudo cmake glfw/
sudo make glfw/build -j8
sudo make glfw/build install
#export https_proxy=http://127.0.0.1:7897