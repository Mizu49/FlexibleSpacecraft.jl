# Data container

Data container is the set of variables that contains all the necessary data of the simulation. We have two sub-modules for data container.

* `Frames`: data container for frame representation
* `TimeLine`: data container for the time-variant physical quantities

## `Frames`

`Frames` is the module that handles data container for attitude frame representation. Please visit [frames](../dynamics/frames.md) for notation and detailed explanation of the attitude frame representation

```@autodocs
Modules = [Frames]
Order   = [:type, :function]
Pages   = ["Frames.jl"]
```

## `TimeLine`

```@autodocs
Modules = [TimeLine]
Order   = [:type, :function]
Pages   = ["TimeLine.jl"]
```