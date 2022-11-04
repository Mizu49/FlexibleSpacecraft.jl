"""
    DataContainers

submodule for accomodating the features and interface functions for handling the data containers for the `FlexibleSpacecraft.jl`.

# Main features

* `function timerange2indexrange`

This submodule also includes the multiple dispatch for the `::AbstractVector{<:AbstractVector}` type data container used for simulation. Please be noted that you may need to pay attention to this feature when you manually code your simulation using the `::AbstractVector{<:AbstractVector}` type variables.
"""
module DataContainers

using StaticArrays

export timerange2indexrange

"""
    timerange2indexrange(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}

returns an `index::::Union{UnitRange{Int64}, Colon}` that corresponding to the given `timerange::Tuple{<:Real, <:Real}`
"""
function timerange2indexrange(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}

    if timerange == (0, 0)
        return :;
    else
        if timerange[1] >= timerange[2]
            throw(DomainError(timerange, "setting for `timerange` is invalid"))
        end

        startindex = convert(Int64, round(timerange[1]/samplingtime) + 1)
        endindex = convert(Int64, round(timerange[2]/samplingtime) + 1)

        return startindex:endindex
    end
end

"""
    timerange2indexrange(timerange::Tuple{<:Real, <:Real}, time::StepRangeLen)::Union{UnitRange{Int64}, Colon}
returns an `index::::Union{UnitRange{Int64}, Colon}` that corresponding to the given `timerange::Tuple{<:Real, <:Real}`
"""
function timerange2indexrange(timerange::Tuple{<:Real, <:Real}, time::StepRangeLen)::Union{UnitRange{Int64}, Colon}

    if timerange == (0, 0)
        return :;
    else
        if timerange[1] >= timerange[2]
            throw(DomainError(timerange, "setting for `timerange` is invalid"))
        end

        samplingtime = convert(Float64, time.step)
        startindex = convert(Int64, round(timerange[1]/samplingtime) + 1)
        endindex = convert(Int64, round(timerange[2]/samplingtime) + 1)

        return startindex:endindex
    end
end

"""
    Base.getindex(v::AbstractVector{<:AbstractVector}, r::AbstractRange, datarow::Int)

get a 1-D subset of the every `datarow`-th value of the inner vector `v::AbstractVector{<:AbstractVector}` within the specified range `r::AbstractRange`.

# Example

```julia
> angularvelocity # ::AbstractVector{<:AbstractVector}
10-element Vector{Vector{Int64}}:
 [1, 2, 3]
 [1, 2, 3]
 ⋮
 [1, 2, 3]
 [1, 2, 3]

> angularvelocity[1]
3-element Vector{Int64}: # get the first vector
 1
 2
 3

> angularvelocity[1:5, 1] # get the 1st element of the 1st to 5th vector of the
 5-element Vector{Int64}:
  1
  1
  1
  1
  1
```
"""
function Base.getindex(v::AbstractVector{<:AbstractVector}, r::Union{AbstractRange, Colon}, datarow::Int)
    return getindex.(v[r], datarow)
end

"""
    Base.getindex(v::AbstractVector{<:AbstractVector}, r::Int, datarow::Int)

get `datarow`-th element of the `r`-th vector in the `v::AbstractVector{<:AbstractVector}`

# Example

```julia
> angularvelocity # ::AbstractVector{<:AbstractVector}
10-element Vector{Vector{Int64}}:
 [1, 2, 3]
 [1, 2, 3]
 ⋮
 [1, 2, 3]
 [1, 2, 3]

> angularvelocity[1]
3-element Vector{Int64}: # get the first vector
 1
 2
 3

> angularvelocity[3, 2] # get the 2st element of the 3rd vector of the `angularvelocity`
2
```

"""
function Base.getindex(v::AbstractVector{<:AbstractVector}, r::Int, datarow::Int)
    return getindex(v[r], datarow)
end

end
