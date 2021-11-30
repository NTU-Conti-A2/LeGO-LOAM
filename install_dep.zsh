mkdir ~/Downloads
# wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.zip
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
# cd ~/Downloads/gtsam-4.0.0-alpha2/
cd ~/Downloads/gtsam-4.0.3/
mkdir build && cd build
apt install gcc-10 g++-10
export CC=gcc-10
export CXX=g++-10
cmake ..
sudo make install