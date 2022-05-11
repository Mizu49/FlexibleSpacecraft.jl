# Example

This documentation page provides an example script and quick start guide for the use of `FlexibleSpacecraft.jl`

## Example script and files

Example script `main.jl` is found in the `/test` directory. Configuration and parameter setting file is preferred for the simulation, and these files should be in YAML format. Detailed formatting for the parameter settings is found in the [Simulation configuration](@ref).

You need the following files:

* [Configuration files for simulation (`simconfig.yml`)](@ref)
* [Dynamics model parameters definition file (`spacecraft.yml`)](@ref)
* [Disturbance parameters (`disturbance.yml`)](@ref)
* [Orbital parameters (`orbit.yml`)](@ref)
* [Initial values (`initvalue.yml`)](@ref)

If you are using CLI, not script, you need one more file:

* [Wrapper for all of the configuration files](@ref)

These files are found on the `test` directory in the GitHub repository. Run the `main.jl`, and you will get the simulation result. By default, the software provides the data set of simulation results and plots of those data. It also gives you a GIF animation of the spacecraft attitude.

## Description of the `main.jl` and UI

This section illustrates the user interface for running the attitude-structure coupling simulation with `FlexibleSpacecraft.jl`. This description is based on the contents in `main.jl`.

Firstly, you need to load the module `FlexibleSpacecraft` into your namespace.

```julia
using FlexibleSpacecraft
```

You need to input the simulation parameters and configuration settings. Our API will help you by loading the parameter setting files:

```julia
# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

# define a orbit info
orbitinfo = setorbit("./test/orbit2.yml", ECI_frame)

# Set disturbance torque
distconfig = setdisturbance("./test/disturbance.yml")

# Initialize the simulation configuration
simconfig = setsimconfig("./test/simconfig.yml")

# Define initial values for simulation
initvalue = setinitvalue("./test/initvalue.yml")
```

Then you are all set! Just run `runsimulation()`. This function is the high-level user interface for simulation. You can find more detailed information at [Simulation interface](@ref)

```julia
# run simulation
println("Begin simulation!")
@time (time, attitudedata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simconfig)
println("Completed!")
```

Congrats! You have successfully run your simulation! Let's process your simulation data. We have covered that for you. Run `quaternion_constraint()` to check your result is physically making sense.

```julia
@test Evaluation.quaternion_constraint(attitudedata.quaternion)
```

Our visualization feature helps you to process your simulation effectively.

```julia
fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity)
# fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity, timerange = (0, 10))
display(fig1)

fig2 = PlotRecipe.quaternions(time, attitudedata.quaternion)
display(fig2)

# Plot of the body frame with respect to ECI frame
fig3 = PlotRecipe.framegif(time, LVLHref, attitudedata.RPYframe, Tgif = 20, FPS = 8)
display(fig3)

# Plot of the euler angle
fig4 = PlotRecipe.eulerangles(time, attitudedata.eulerangle)
display(fig4)
```

`FlexibleSpacecraft.jl` also helps you to save the simulation data as CSV files. Specify the directory you want to save your data. The simulation result will be saved as a directory with timestamp.

```julia
location = "output" # specify where to save your data
outputdata = SimData(time, attitudedata, orbitdata)
write(location, outputdata)
```
