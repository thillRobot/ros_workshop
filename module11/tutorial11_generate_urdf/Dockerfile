FROM ros:noetic-robot-focal

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt update && apt install -y apt-utils build-essential psmisc vim-gtk git 

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# RUN apt install -y ros-noetic-tf ros-noetic-xacro

RUN apt install -y ros-noetic-rviz ros-noetic-joint-state-publisher-gui


ENV GURDF_WS=/generate_urdf_workspace

RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p $GURDF_WS/src && \
    cd $GURDF_WS/src && \
    git clone https://github.com/thillRobot/generate_urdf.git && \
    cd $GURDF_WS && \
    catkin_make

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

COPY ./generate_urdf_entrypoint.sh /

ENTRYPOINT ["/generate_urdf_entrypoint.sh"]

CMD ["bash"]

