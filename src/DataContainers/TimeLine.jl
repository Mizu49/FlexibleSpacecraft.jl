"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine

using StaticArrays

export getdataindex

"""
    function getdataindex(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}

returns an `index::::Union{UnitRange{Int64}, Colon}` that corresponding to the given `timerange::Tuple{<:Real, <:Real}`
"""
function getdataindex(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}

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

end
