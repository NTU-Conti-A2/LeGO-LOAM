# syntax=docker/dockerfile:1

# seems to result in pcl-1.10 being installed, which is pretty old.

# FROM ros:noetic-robot
FROM osrf/ros:noetic-desktop-full
# FROM osrf/ros:melodic-desktop-full-bionic 
# FROM osrf/ros:melodic-desktop-full


# apparently this will stop errors such as certain apt install commands needing you to specify the nationality of your keyboard
ARG DEBIAN_FRONTEND=noninteractive

# install useful things
RUN apt-get update && apt-get install -y wget git vim unzip libparmetis-dev gcc-10 g++-10

ENV CC=gcc-10
ENV CXX=g++-10
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# install zsh
RUN apt-get update && apt-get install -y zsh && \
    chsh -s $(which zsh) && \
    sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# tell Docker to use zsh as it builds. Probably not important.
SHELL ["/usr/bin/zsh","-c"]

# configure zsh. add any desired pluigns here.
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc && \
    sed -i 's/ZSH_THEME=\"robbyrussell\"/ZSH_THEME=\"ys\"/' ~/.zshrc && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions extract web-search zsh-syntax-highlighting)/ '   ~/.zshrc 

# # setup python stuff. Not needed for Ubuntu-Bionic. Note that the ros image does not come with python2.
RUN apt install -y python3-pip python-is-python3

# pytorch stuff. Note that GPU is NOT enabled in this version. see https://pytorch.org/get-started/locally/
RUN pip3 install torch==1.9.0+cpu torchvision==0.10.0+cpu torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html

# # misc ROS stuff, particularly for visualization. Note the container must also be run with the needed WSLG stuff. See the README.md
# RUN apt update && apt install ros-noetic-rviz ros-noetic-image-transport-plugins \
#     ros-noetic-tf2-tools ros-noetic-rqt-graph ros-noetic-rqt-common-plugins -y

# # hdl-graph-slam stuff
# RUN apt update && apt install -y ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o
# # note the library still has to be compiled, I did that after running the container.

RUN pip install python-igraph

# LeGO-LOAM Stuff

## install LeGO-LOAM dependencies - GTSAM
WORKDIR /tmp
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.zip
RUN unzip gtsam.zip
RUN cd gtsam-4.0.3 && mkdir build && cd build && cmake .. && make install -j$(nproc)
RUN rm -rf gtsam-4.0.3 gtsam.zip

# create a catkin_ws
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    source /opt/ros/noetic/setup.zsh && \
    catkin_make

# Compile LeGO-LOAM
WORKDIR /root
# RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
# Copying NTU LeGO-LOAM repo into docker build context (must build in parent directory "docker build -t ntu-lego-loam:latest -f docker/Dockerfile .")
COPY . /root/catkin_ws/src/LeGO-LOAM
RUN /usr/bin/zsh -c 'source /opt/ros/noetic/setup.zsh; cd ~/catkin_ws ; catkin_make -j4'
# Removing source file as will be bind-mounting in docker-compose
RUN rm -rf /root/catkin_ws/src/LeGO-LOAM

# enable auto overlaying our workspace
RUN echo "source /root/catkin_ws/devel/setup.zsh" >> ~/.zshrc

# overwrites the entrypoint the ros-images had set
ENTRYPOINT []
CMD ["/bin/zsh"]

# these aren't quite working. would be nice to always launch into the workspace.
# CMD cd /root/catkin_ws && zsh 
# CMD ["zsh"]