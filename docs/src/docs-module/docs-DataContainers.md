# Data container and its handling interfaces

## `Frames`

`Frames` is the module that handles data container for attitude frame representation. Please visit [frames](../dynamics/frames.md) for notation and detailed explanation of the attitude frame representation

```@autodocs
Modules = [Frames]
Order   = [:type, :function]
Pages   = ["Frames.jl"]
```

## `DataContainers`

`DataContainers.jl` is the set of function that deals with the handling of all the necessary data container of the simulation. 

!!! note "Dispatch of the `::AbstractVector{<:AbstractVector}`"
    This submodule also includes the multiple dispatch for the `::AbstractVector{<:AbstractVector}` type data container used for simulation. Please be noted that you may need to pay attention to this feature when you manually code your simulation using the `::AbstractVector{<:AbstractVector}` type variables.

```@autodocs
Modules = [DataContainers]
Order   = [:type, :function]
Pages   = ["DataContainers.jl"]
```
