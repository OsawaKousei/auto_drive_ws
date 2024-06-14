FROM n622jwith/ros2_auto_drive_base:latest

# Add user and group, then allow sudo
ARG UID
ARG GID
ARG PASSWORD
ARG USER_NAME
ARG GROUP_NAME
ARG GIT_USER_NAME
ARG GIT_USER_EMAIL

RUN groupadd -g ${GID} ${GROUP_NAME}
RUN useradd -u ${UID} -g ${GID} -G sudo -s /bin/bash -m ${USER_NAME}
RUN echo "${USER_NAME}:${PASSWORD}" | chpasswd
RUN echo "Defaults visiblepw" >> /etc/sudoers
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER_NAME}

# git config
RUN git config --global user.name "${GIT_USER_NAME}" && \
    git config --global user.email "${GIT_USER_EMAIL}"

# setup ros2
COPY setup.sh /home/${USER_NAME}/setup.sh
RUN bash ~/setup.sh

CMD ["/bin/bash"]
