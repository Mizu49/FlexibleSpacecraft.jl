# NOTE
#   This file should be included at the last of `FlexibleSpacecraft.jl`.

using Comonicon
using Pkg
using YAML

"""
Run simulation with FlexibleSpacecraft.jl according to given configuration files.

# Args

* `configfilepath::String`: path for the representative configulation file (YAML)

"""
@cast function run(configfilepath::String)

    print("loading the simulation configulation files...")

    # Read the YAML file to obtain the pathes for the configulation files
    configfiles = YAML.load_file(configfilepath)

    # Set the dynamics model
    model = setdynamicsmodel(configfiles["configfiles"]["model"],)

    # define a orbit info
    orbitinfo = setorbit(configfiles["configfiles"]["orbit"], ECI_frame)

    # Set disturbance torque
    distconfig = setdisturbance(configfiles["configfiles"]["disturbance"])

    # Initialize the simulation configuration
    simconfig = setsimconfig(configfiles["configfiles"]["simconfig"])

    # Define initial values for simulation
    initvalue = setinitvalue(configfiles["configfiles"]["initvalue"])

    println("done!")

    # Execute the simulation
    print("running simulation ...")
    (time, attitudedata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)
    println("completed!")

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
