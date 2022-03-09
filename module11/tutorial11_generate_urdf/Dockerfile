FROM osrf/ros:noetic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim-gtk git 

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# RUN apt-get update && apt-get install -q -y python-catkin-tools

ENV GURDF_WS=/generate_urdf_workspace

RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p $GURDF_WS/src && \
    cd $GURDF_WS/src && \
    catkin_init_workspace && \
    git clone https://github.com/thillRobot/generate_urdf.git -b noetic && \
    cd $GURDF_WS && \
    catkin_make && \
    catkin_make install

EXPOSE 11345

COPY ./generate_urdf_entrypoint.sh /

ENTRYPOINT ["/generate_urdf_entrypoint.sh"]

CMD ["bash"]

