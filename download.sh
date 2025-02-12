#!/bin/bash

# Dataset 디렉토리 생성 및 이동
mkdir -p Dataset/mesh Dataset/point
cd Dataset

# Mesh 데이터 다운로드 및 정리
gdown 1org3At8iT-t3fB3CTgdvQsZ_fO6RwYE1 -O Mesh.zip
unzip Mesh.zip -d Mesh
rm -rf __MACOSX

# Point 데이터 다운로드 및 정리
gdown 1C03oiRGsIiFE1-jUWDZypjdqo0fDvwqS -O Point.zip
unzip Point.zip -d Point
rm -rf __MACOSX

# Mesh 데이터 정리
find mesh -type f -name "*.obj" -exec mv {} mesh/ \;

# Point 데이터 정리
find point -type f -name "*.pcd" -exec mv {} point/ \;

# 불필요한 압축 파일 삭제
rm -rf Mesh.zip Point.zip

# 상위 디렉토리로 이동
cd ..
