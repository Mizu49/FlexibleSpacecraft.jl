# Disturbance input to attitude dynamics

Disturbance input for attitude motion is crucial to model how the spacecraft attitude changes from the effect of the outer environment. A submodule `AttitudeDisturbance.jl` offers a disturbance input generation feature for `FlexibleSpacecraft.jl`.

## Disturbance input models

* Constant torque input
* Step disturbance input trajectory
* Gravitational torque input

## Parameter settings for disturbance input

You need an parameter setting YAML file to configure the disturbance input to the spacecraft. The file is presented as follows:

```yaml
disturbance:
    constant torque: [0.0, 0.0, 0.0]
    gravitational torque: false
    step trajectory:
        value: [[10, 0, 0], [0, 0, 0]]
        endurance: [1, 100]
```

## Libraries for `AttitudeDisturbance.jl`

```@autodocs
Modules = [AttitudeDisturbance]
Order   = [:type, :function]
Pages   = ["AttitudeDisturbance.jl"]
```
