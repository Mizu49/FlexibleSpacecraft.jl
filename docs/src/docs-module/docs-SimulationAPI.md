# Simulation API

This section contains information about the API for the simulation system of `FlexibleSpacecraft.jl`

Our API mainly contains the following sub-modules:

* `ParameterSetting`: API for parameter setting for simulation core

The high-level API function for the simulation is `runsimulation`. Basically you can run your simulation by passing all the necessary arguments into function `runsimulation`.

## `ParameterSetting`

```@autodocs
Modules = [ParameterSetting]
Order   = [:type, :function]
Pages   = ["ParameterSetting.jl"]
```
