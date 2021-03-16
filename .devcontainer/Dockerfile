FROM julia
USER root

RUN apt-get update && apt-get install -y git x11-apps xauth 

RUN julia -e 'using Pkg; Pkg.REPLMode.pkgstr("add Plots;precompile");using Plots'
