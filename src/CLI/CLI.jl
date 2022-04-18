# NOTE
#   This file should be included at the last of `FlexibleSpacecraft.jl`.

using Comonicon
using Pkg

"""
Run simulation with FlexibleSpacecraft.jl according to given configuration files.

# Args

- `config_model`: Path to a configuration file for the dynamics model.
- `config_orbid`: Path to a configuration file defining orbit information.
- `time_simulation`: Time length of the simulation.

# Options

- `--period-sampling`: Sampling period of the simulation (seconds).

# Flags

- `--no-gravitygradient`: Disable gravitational torques in the simulation.
"""
@cast function run(
    config_model::String,
    config_orbid::String,
    time_simulation::Float64;
    period_sampling::Float64 = 1e-2,
    no_gravitygradient::Bool = false,
)
    (simconfig, ECI_frame) = initsimulation(time_simulation, period_sampling)
    model = setdynamicsmodel(config_model)
    orbitinfo = initorbitinfo(config_orbid, ECI_frame)
    distconfig = DisturbanceConfig(gravitygradient = !no_gravitygradient)

    initvalue = TimeLine.InitData(
        [0, 0, 0, 1],
        [0, 0, 0],
        ECI_frame
    )

    @time (time, simdata, orbitdata) = runsimulation(
        model,
        ECI_frame,
        initvalue,
        orbitinfo,
        distconfig,
        simconfig,
    )
end

"""
Update and re-build FlexibleSpacecraft.
"""
@cast function update()
    Pkg.update("FlexibleSpacecraft")
    Pkg.build("FlexibleSpacecraft")
end

"""
Clear FlexibleSpacecraft.
"""
@cast function clear()
    Pkg.rm("FlexibleSpacecraft")
end

"""
CLI tool for FlexibleSpacecraft.
"""
@main
