FROM osrf/ros:noetic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim-gtk git 

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# RUN apt-get update && apt-get install -q -y python-catkin-tools

# Install git lfs. Necessary in order to properly clone bobble_description
# RUN echo 'deb http://http.debian.net/debian wheezy-backports main' > /etc/apt/sources.list.d/wheezy-backports-main.list
# RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
# RUN apt-get install -q -y git-lfs
# RUN git lfs install

ENV MISSIM_WS=/mission_simulator_workspace

RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p $MISSIM_WS/src && \
    cd $MISSIM_WS/src && \
    catkin_init_workspace && \
    git clone https://github.com/thillRobot/generate_urdf.git && \
    cd $MISSIM_WS && \
    catkin_make && \
    catkin_make install

# Required python packages for analysis
#RUN \
#  apt-get install -y python-pip && \
#  pip install matplotlib && \
#  pip install numpy && \
#  pip install scipy && \
#  pip install jupyter && \
#  pip install seaborn && \
#  pip install pandas && \
#  pip install bokeh && \
#  pip install vcstool && \
#  pip install rosbag_pandas

EXPOSE 11345

COPY ./mission_simulator_entrypoint.sh /

ENTRYPOINT ["/mission_simulator_entrypoint.sh"]

CMD ["bash"]