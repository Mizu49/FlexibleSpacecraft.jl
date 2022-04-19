# NOTE
#   This file should be included at the last of `FlexibleSpacecraft.jl`.

using Comonicon
using Pkg

"""
Run simulation with FlexibleSpacecraft.jl according to given configuration files.

# Args

- `configfilepath_model`: Path to a configuration file for the dynamics model.
- `configfilepath_orbit`: Path to a configuration file defining orbit information.
- `configfilepath_disturbance`: Path to the configuration file for the disturbance input
- `configfilepath_simulation`: Path to the configuration file for the simulation settings
- `configfilepath_initvalue`: Path to the configuration file for the initial value of the simulation

"""
@cast function run(
    configfilepath_model::String,
    configfilepath_orbit::String,
    configfilepath_disturbance::String,
    configfilepath_simulation::String,
    configfilepath_initvalue::String
)

    println("run simulation ...")

    # Set the dynamics model
    model = setdynamicsmodel(configfilepath_model,)

    # define a orbit info
    orbitinfo = setorbit(configfilepath_orbit, ECI_frame)

    # Set disturbance torque
    distconfig = setdisturbance(configfilepath_disturbance)

    # Initialize the simulation configuration
    simconfig = setsimconfig(configfilepath_simulation)

    # Define initial values for simulation
    initvalue = setinitvalue(configfilepath_initvalue)

    (time, attitudedata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)

    println("Simulation completed!")
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
