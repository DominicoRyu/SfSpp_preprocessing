# 1. Base Image 설정
FROM ubuntu:20.04

# 2. 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PCL_VERSION="1.9.1"
ENV CGAL_VERSION="5.0"
ENV VTK_VERSION="8.1.2"

# 3. 필수 패키지 및 종속성 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    g++ \
    cmake \
    git \
    wget \
    qt5-default \
    libqt5opengl5-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    mesa-common-dev \
    libqt5svg5-dev \
    qttools5-dev \
    zlib1g-dev \
    libusb-1.0-0-dev \
    libpng-dev \
    libeigen3-dev \
    libflann-dev \
    libqhull-dev \
    libgmp-dev \
    libmpfr-dev \
    libx11-dev \
    libxt-dev \
    libxext-dev \
    libxrender-dev \
    x11-apps \
    libboost-all-dev \
    python3-pip \
    unzip \
    libxmu-dev \
    libxi-dev \
    libosmesa6-dev \
    software-properties-common \
    --fix-missing \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 5. VTK 8.1.2 다운로드 및 빌드
WORKDIR /tmp
RUN wget https://www.vtk.org/files/release/8.1/VTK-${VTK_VERSION}.tar.gz \
    && tar -xvzf VTK-${VTK_VERSION}.tar.gz \
    && mv VTK-${VTK_VERSION} vtk-src \
    && mkdir -p vtk-src/build \
    && cd vtk-src/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_SHARED_LIBS=ON \
        -DVTK_GROUP_ENABLE_Qt=NO \
        -DVTK_USE_X=ON \
        -DVTK_RENDERING_BACKEND=OpenGL2 \
        -DVTK_MODULE_ENABLE_VTK_RenderingCore=YES \
        -DVTK_OPENGL_HAS_OSMESA=ON \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && cd /tmp \
    && rm -rf vtk-src VTK-${VTK_VERSION}.tar.gz

# 6. PCL 소스 다운로드 및 빌드
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${PCL_VERSION}.tar.gz \
    && tar -xvzf pcl-${PCL_VERSION}.tar.gz \
    && mv pcl-pcl-${PCL_VERSION} pcl-src \
    && mkdir -p pcl-src/build \
    && cd pcl-src/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_CUDA=OFF \
        -DBUILD_examples=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_apps=OFF \
        -DBUILD_surface=ON \ 
        -DBUILD_surface_on_nurbs=ON \
        -DBUILD_features=ON \ 
        -DWITH_VTK=ON \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) \
    && make install \
    && cd /tmp \
    && rm -rf pcl-src pcl-${PCL_VERSION}.tar.gz

# 7. CGAL 소스 다운로드 및 빌드
RUN wget https://github.com/CGAL/cgal/releases/download/releases/CGAL-5.0/CGAL-5.0.tar.xz \
    && tar -xvJf CGAL-5.0.tar.xz \
    && mv CGAL-5.0 cgal-src \
    && mkdir -p cgal-src/build \
    && cd cgal-src/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) \
    && make install \
    && cd /tmp \
    && rm -rf cgal-src CGAL-5.0.tar.xz

# 7. tiny_obj_loader.h 다운로드 및 설치
RUN pip3 install gdown \
    && gdown --id 1b3OAIWT_AVGi0Uf4BRuE7TNvli_MFnAf -O tiny_obj_loader.h \
    && mv tiny_obj_loader.h /usr/local/include/