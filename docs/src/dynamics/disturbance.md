# Disturbance input to attitude dynamics

Disturbance input for attitude motion is crucial to model how the spacecraft attitude changes from the effect of the outer environment. `FlexibleSpacecraft.jl` offers a disturbance input generation feature.

## Disturbance input models

* Constant torque input
* Gravitational torque input

## Parameter settings for disturbance input

You need an parameter setting YAML file to configure the disturbance input to the spacecraft. The file is presented as follows:

```yaml
# set `property` distconfig to indicate that this file configures the disturbance input
property: distconfig

# set constant torque input with 3-d vector, define it to be [0, 0, 0] if no constant torque is applied
constant torque: [0.05, 0, 0]
# Boolean to select if the gravitational torque is considered or not.
gravitational torque: false
```

## Libraries

```@autodocs
Modules = [Disturbance]
Order   = [:type, :function]
Pages   = ["Disturbance.jl"]
```
