"""
    Base.getindex(v::AbstractVector{<:AbstractVector}, r::AbstractRange, datarow::Int)

get a 1-D subset of the every `datarow`-th row of `v::AbstractVector{<:AbstractVector}` within `r::AbstractRange`, used for custom data container for `FlexibleSpacecraft.jl`
"""
function Base.getindex(v::AbstractVector{<:AbstractVector}, r::Union{AbstractRange, Colon}, datarow::Int)
    return getindex.(v[r], datarow)
end

"""
    Base.getindex(v::AbstractVector{<:AbstractVector}, r::Int, datarow::Int)

get an element of the `v<:SVector`, used for custom data container for `FlexibleSpacecraft.jl`
"""
function Base.getindex(v::AbstractVector{<:AbstractVector}, r::Int, datarow::Int)
    return getindex(v[r], datarow)
end
