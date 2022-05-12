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
