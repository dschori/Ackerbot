# Dockerfile for tensorflow 2 with gpu
FROM nvidia/cuda:11.1-base-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
          nano \
	  cmake \
	  build-essential \
	  curl \
	  wget \
	  gnupg2 \
	  lsb-release \
	  python3-pip \
      python3-dev \
      python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Install OpenJDK-8
RUN apt-get update && \
    apt-get install -y openjdk-8-jdk && \
    apt-get install -y ant && \
    apt-get clean;

# Fix certificate issues
RUN apt-get update && \
    apt-get install ca-certificates-java && \
    apt-get clean && \
    update-ca-certificates -f;

RUN apt update && \
    apt install -y snapd;

# Setup JAVA_HOME -- useful for docker commandline
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64/
RUN export JAVA_HOME




FROM ubuntu:latest
RUN apt-get -y update
RUN apt-get -y install firefox
RUN apt-get -y install xauth
