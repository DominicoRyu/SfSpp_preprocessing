#!/bin/bash

# DISPLAY 환경 변수 설정
export DISPLAY=172.18.224.1:0

# xhost 설정
xhost + 172.18.224.1

# Docker 컨테이너 실행
docker run \
    -it \
    --rm \
    -e DISPLAY=$DISPLAY \
    -v "$HOME/.Xauthority:/home/user/.Xauthority" \
    -v "/home/liuss98/Drive/Test/Pre-processing:/SfS" \
    sfs_pre:latest
