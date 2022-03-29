# Example

This documentation page is an example and quick start guide for the use of `FlexibleSpacecraft.jl`

## Example script and files

Example script `main.jl` is found in the `/test` directory. Configuration and parameter setting file is preferred for the simulation, and these files should be in YAML format. Detailed formatting for the parameter settings is found in the [Parameter configurations]().

You need the following files:

* `main.jl`: the main script
* `orbit.yml`: main configuration for orbital parameters
* `spacecraft.yml`: parameter settings for spacecraft configuration

These files are found on the `test` directory in the GitHub repository. Run the `main.jl`, and you will get the simulation result. By default, the software provides the data set of simulation results and plots of those data. It also gives you a GIF animation of the spacecraft attitude.

## Description of the `main.jl` and UI

This section illustrates the user interface for running the attitude-structure coupling simulation with `FlexibleSpacecraft.jl`. This description is based on the contents in `main.jl`.

Firstly, you need to load the module `FlexibleSpacecraft` into your namespace.

```julia
using FlexibleSpacecraft
```

Then you need to configure the simulation. 

```julia
# Sampling period of simulation (second)
Tsampling = 1e-2
# Time length of simulation (second)
simulation_time = 1000

# Initialize the simulation configurations
(simconfig, ECI_frame) = initsimulation(simulation_time, Tsampling)
```

Then you need to define the dynamics model of spacecraft and orbit. And other parameters for simulation are defined at this point.

```julia
# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

# define a orbit info
orbitinfo = initorbitinfo("./test/orbit.yml", ECI_frame)

# Set disturbance torque
distconfig = DisturbanceConfig(gravitygradient = true)
```

Next, initialize the time-varying states and give all the data to the simulation API.

```julia
# Initialize data array
initvalue = TimeLine.InitData(
    [0, 0, 0, 1],
    [0, 0, 0],
    ECI_frame
)
```

Then you are all set! Just run `runsimulation()`. This function is the high-level user interface for simulation. 

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
fig3 = PlotRecipe.framegif(time, ECI_frame, attitudedata.bodyframe, Tgif = 20, FPS = 8)
display(fig3)
```
