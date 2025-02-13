# Image for building Stratux
#
FROM debian:bookworm

# file and nano are nice to have
RUN apt-get update \
  && apt-get -y install file \
  && apt-get -y install nano \
  && apt-get -y install make \
  && apt-get -y install git \
  && apt-get -y install gcc \
  && apt-get -y install ncurses-dev \
  && apt-get -y install golang-go \
  && apt-get -y install wget \
  && apt-get -y install libusb-1.0-0-dev

RUN cd /tmp \
    && wget https://github.com/stratux/rtlsdr/releases/download/v1.0/librtlsdr0_2.0.2-2_arm64.deb \
    && dpkg -i librtlsdr0_2.0.2-2_arm64.deb

RUN cd /tmp \
    && wget https://github.com/stratux/rtlsdr/releases/download/v1.0/librtlsdr-dev_2.0.2-2_arm64.deb \
    && dpkg -i librtlsdr-dev_2.0.2-2_arm64.deb

# specific to debian, ubuntu images come with user 'ubuntu' that is uid 1000
ENV USERNAME="stratux"
ENV USER_HOME=/home/$USERNAME

RUN useradd -m -d $USER_HOME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
