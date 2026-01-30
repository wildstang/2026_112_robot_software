FROM mcr.microsoft.com/devcontainers/java:1-17-bookworm

RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | gpg --dearmor > /usr/share/keyrings/yarn-archive-keyring.gpg

# install dependencies
RUN apt update
RUN apt install -y python3-pip git-lfs
RUN pip3 install --break-system-packages requests

# download WPILib extension
COPY ./get-wpilib.py /opt
RUN python3 /opt/get-wpilib.py