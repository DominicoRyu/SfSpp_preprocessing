#!/bin/bash

mkdir -p Dataset
cd Dataset

gdown 1m7VEDaX6bdyOjGoD6JmK4bwoxSQ-EeKl -O Mesh.zip
unzip Mesh.zip
rm -rf __MACOSX Mesh.zip 

gdown 1bBuqBIFnOQug9O0aYTkYbDwzIAaGMTsx -O Point.zip
unzip Point.zip
rm -rf __MACOSX Point.zip  

cd ..
