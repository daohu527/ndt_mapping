FROM ubuntu:18.04
ARG DEBIAN_FRONTEND=noninteractive

COPY sources.list.aliyun /etc/apt/sources.list

RUN apt-get update && apt-get install -y \
    libgflags-dev \
    libpcl-dev \
    libeigen3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/include/pcl-1.8/pcl /usr/include/pcl
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
RUN ln -s /usr/include/eigen3/unsupported /usr/include/unsupported

# bazel
RUN apt-get update && apt-get install -y \
    g++ \
    unzip \
    zip \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ARG BAZEL_VERSION=5.3.0
# ARG BAZEL_INSTALL_FILE=bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh

# RUN wget https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${BAZEL_INSTALL_FILE}

COPY bazel-5.3.0-installer-linux-x86_64.sh .

RUN chmod +x bazel-5.3.0-installer-linux-x86_64.sh \
    && ./bazel-5.3.0-installer-linux-x86_64.sh --user \
    && rm -rf bazel-5.3.0-installer-linux-x86_64.sh

ENV PATH="${PATH}:/root/bin"

WORKDIR "/home"
