# NOTE
#   This file should be included at the last of `FlexibleSpacecraft.jl`.

using Comonicon
using Pkg
using YAML

"""
Run simulation with FlexibleSpacecraft.jl according to given configuration files.

# Args

* `configfilepath`: path for the representative configulation file (YAML)

# Flags

* `--save`: an option to save the data, false by default

"""
@cast function run(configfilepath::String; save::Bool = false)


    # Read the YAML file to obtain the pathes for the configulation files
    print("loading the simulation configulation files...")
    (simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel) = readparamfile(configfilepath)
    println("completed!")

    # run simulation
    print("Begin simulation...")
    @time (time, attitudedata, orbitdata, strdata) = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, simconfig)
    println("completed!")

    if save == true
        print("saveing data...")

        outputdata = SimData(time, attitudedata, orbitdata)
        write("../output", outputdata)
    end

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
