sudo apt-get install cmake xorg-dev libglu1-mesa-dev


sudo mkdir glfw/build
sudo cmake glfw/
sudo make glfw/build -j8
sudo make glfw/build install