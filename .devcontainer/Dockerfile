FROM julia
USER root

RUN apt-get update && apt-get install -y git x11-apps

# Required package for GR backend of Plots
# https://gr-framework.org/julia.html#installation
RUN apt-get install -y libxt6 libxrender1 libxext6 libgl1-mesa-glx libqt5widgets5


RUN julia -e 'using Pkg; Pkg.REPLMode.pkgstr("add Plots;precompile");using Plots'


ENV DISPLAY host.docker.internal:0.0
ENV XDG_RUNTIME_DIR /workspaces/Julia-programing/.tmp
ENV LIBGL_ALWAYS_INDIRECT 1
