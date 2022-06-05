# Simulation API

This section contains information about the API for the simulation system of `FlexibleSpacecraft.jl`

Our API mainly contains the following sub-modules:

* `ParameterSettingBase`: API for parameter setting for simulation core

The high-level API function for the simulation is `runsimulation`. Basically you can run your simulation by passing all the necessary arguments into function `runsimulation`.

## `ParameterSettingBase`

```@autodocs
Modules = [ParameterSettingBase]
Order   = [:type, :function]
Pages   = ["ParameterSettingBase.jl"]
```
