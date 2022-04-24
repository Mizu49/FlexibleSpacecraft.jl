# Attitude dynamics general

Internal implementation of the attitude dynamics model in `FlesibleSpacecraft.jl` is found on `src/AttitudeDynamics`.

* `Attitude.jl`: common features and implementation of attitude dynamics
* `Evaluation.jl`: evaluation features of attitude dynamics
* `RigidBody.jl`: rigid body dynamics model

This section provides information of `Attitude.jl`. This submodule deals with the common features in the attitude dynamics calculation.

```@docs
Attitude
```

## functions

```@autodocs
Modules = [Attitude]
Order   = [:type, :function]
Pages   = ["Attitude.jl"]
```
