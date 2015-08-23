FROM ubuntu:12.04
MAINTAINER Amit Bakshi <ambakshi@gmail.com>

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -yq
RUN apt-get install -yq python-software-properties
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update -yq
RUN apt-get install -yq build-essential gcc-4.8 g++-4.8 cmake make bison flex libpthread-stubs0-dev
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.6
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 40 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8
RUN echo 2 | update-alternatives --config gcc
ADD . /app
WORKDIR /app/build
RUN cmake -G 'Unix Makefiles' ..
CMD ["/usr/bin/make","-j2"]
