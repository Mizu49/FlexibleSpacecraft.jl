# Documentation for the CLI

`FlexibleSpacecraft.jl` offers a useful CLI tools to accelerate your development of the spacecraft. This documentation illustrates the basic settings and configurations of the CLI tool.

## Pre-process of the simulation parameters

You need to prepare all of the parameters for simulation in the following manner.

You need to an YAML file that specifies the locaton of the parameter setting files for the each subsystem.

* `spacecraft.yml`: parameters for the spacecraft itself and its model formulation
* `orbit.yml`: parameters for the orbital motion
* `disturbance.yml`: parameters for the disturbance input
* `simconfig.yml`: configuration files for the simulation
* `initvalue.yml`: configuration of initial value for the simulation

The location of these files should be addressed in the YAML file like:

```yaml
name: Test parameters # Name of the parameters setting
dates: "2022/04/20" # Date (Optional)
notes: "This YAML file is only for testing the CLI" # Notes (Optional)

# Specify the relative path of the parameter configuration files (Required) 
# These file locations should be declared in this file, otherwise, the software gives an error
configfiles:
    model: "spacecraft.yml"
    orbit: "orbit2.yml"
    disturbance: "disturbance.yml"
    simconfig: "simconfig.yml"
    initvalue: "initvalue.yml"
```

Save this file as YAML files like `params.yml`. In the following documentation, we will use `params.yml`.

## Basic commands

`evalspacecraft` is the topmost basic CLI command for `FlexibleSpacecraft.jl`.

