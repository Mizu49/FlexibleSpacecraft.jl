# Simulation configuration

`FlexibleSpacecraft.jl` offers an soft cording method to allow you to manage your simulation parameters and configurations. You need parameter setting and simulation configuration files to run your simulation with your desired setting. This documentation illustrates how to prepare your simulation configuration files.

These files are organized with the YAML format. For more information about YAML, please visit [The Official YAML Web Site](https://yaml.org/).

## Configuration files for simulation (`simconfig.yml`)

`simconfig.yml` is a configuration setting files for the simulation. You need to define the following fields:

* sampling time: sampling time of your simulation (unit: seconds)
* simulation time: how long you want to simulate (unit: seconds)

The YAML format is:

```yaml
# Specify property of this YAML file
property: simconfig

# Sampling period of simulation (seconds) [Required]
sampling time: 1e-2

# Time length of simulation (seconds) [Required]
simulation time: 1000
```

## Dynamics model parameters definition file (`spacecraft.yml`)

This files addresses the basic configuration for the dynamics model and its parameters of your spacecraft configurations. This parameters are unique based on what type of model you might want to use. For more detailed information on how to define parameter settings for a specific dynamics model, please visit the docs for each model:

* [Parameter settings for rigid body attitude dynamics](@ref)
* *Docs to be updated*
* *Docs to be updated*

## Disturbance parameters (`disturbance.yml`) 

`disturbance.yml` is a parameter setting files for disturbance input of the spacecraft. Please visit the docs for disturbance input listed below:

* *Docs to be updated*
* *Docs to be updated*

## Orbital parameters (`orbit.yml`)

`orbit.yml` is a parameter setting file for the orbital dynamics. Please visit the docs for orbital dynamics for the detailed setting and preparation for this file.

## Initial values (`initvalue.yml`)

`initvalue.yml` is a parameter setting file for the initial value for simulation. You need to define the parameter as follows:

```yaml
# Specify property of this YAML file
property: initvalue

quaternion: [0, 0, 0, 1]
angular velocity: [0, 0, 0]
```

## Wrapper for all of the configuration files.

The provided CLI feature uses a YAML file that indicates the relative path for all of the above configuration files:

```yaml
# Parameters setting files for the FlexibleSpacecraft.jl
name: Test parameters
dates: "2022/04/20"
notes: "This YAML file is only for testing the CLI"

# Relative path of the configuration files (Required)
configfiles:
    model: "spacecraft.yml"
    orbit: "orbit2.yml"
    disturbance: "disturbance.yml"
    simconfig: "simconfig.yml"
    initvalue: "initvalue.yml"
```

This file is used for CLI feature. See [Documentation for the CLI](@ref) for more information
