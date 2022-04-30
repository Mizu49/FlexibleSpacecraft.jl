# Get official docker image fo julia
FROM julia

# set username as `root`
USER root

# update package list
RUN apt-get update

# install git
RUN apt-get install -y git

# Required package for GR backend of Plots
# https://gr-framework.org/julia.html#installation
RUN apt-get install -y libxt6 libxrender1 libxext6 libgl1-mesa-glx libqt5widgets5

# Required in the build process
RUN apt-get install unzip

ENV PATH="/root/.julia/bin:$PATH"
