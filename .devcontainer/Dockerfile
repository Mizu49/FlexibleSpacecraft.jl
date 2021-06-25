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

# Install `Plots.jl and precompile it`
RUN julia -e 'using Pkg; Pkg.REPLMode.pkgstr("add Plots;precompile");using Plots'

# Install `Reexport.jl`
RUN julia -e 'import Pkg; Pkg.add("Reexport")'

# Install `Documenter.jl`
RUN julia -e 'import Pkg; Pkg.add("Documenter")'

# Install `LiveServer.jl` to serve the documentation locally
RUN julia -e 'import Pkg; Pkg.add("LiveServer")'
