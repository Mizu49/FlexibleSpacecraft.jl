"""
    DataAPI

API submodule for handling the `DataFrame` of the simulation results
"""
module DataAPI

using DataFrames, CSV, StaticArrays, StructArrays
using ..Frames, ..TimeLine

export SimData

"""
    SimData

struct of the table data frame from simulation results

# Fields

* `attitude::DataFrame`: tabobject for the attitude data
* `orbit`: `DataFrame` object for the orbit data

# Usage

use the following constructor:
`SimData(time::AbstractRange{<:Real}, attitudedata::StructVector, orbitdata::StructVector)`

* `time`: time data
* `attitudedata`: `StructVector` of the attitude data
* `orbitdata`: `StructVector` of the orbit data
"""
struct SimData

    attitude::DataFrame
    orbit::DataFrame

    # Constructor
    SimData(time::AbstractRange{<:Real}, attitudedata::StructVector, orbitdata::StructVector) = begin

        attitude = _convert_attitude(time, attitudedata)
        orbit = _convert_orbit(time, orbitdata)

        new(attitude, orbit)
    end
end

"""
    _convert_attitude(time::AbstractRange{<:Real}, attitudedata::StructVector)::DataFrame

convert internal data container for attitude to the data frame object by mapping each fields
"""
function _convert_attitude(time::AbstractRange{<:Real}, attitudedata::StructVector)::DataFrame

    attitude = DataFrame(
        "time" => time,
        "quaternion 1" => attitudedata.quaternion[:, 1],
        "quaternion 2" => attitudedata.quaternion[:, 2],
        "quaternion 3" => attitudedata.quaternion[:, 3],
        "quaternion 4" => attitudedata.quaternion[:, 4],
        "angular velocity 1" => attitudedata.angularvelocity[:, 1],
        "angular velocity 2" => attitudedata.angularvelocity[:, 2],
        "angular velocity 3" => attitudedata.angularvelocity[:, 3],
        "roll" => attitudedata.eulerangle[:, 1],
        "pitch" => attitudedata.eulerangle[:, 2],
        "yaw" => attitudedata.eulerangle[:, 3],
    )

    return attitude
end

"""
    _convert_orbit(time::AbstractRange{<:Real}, orbitdata::StructVector)::DataFrame

convert internal data container for orbital motion to the data frame object by mapping each fields
"""
function _convert_orbit(time::AbstractRange{<:Real}, orbitdata::StructVector)::DataFrame

    orbit = DataFrame(
        "time" => time,
        "angular position" => orbitdata.angularposition,
        "angular velocity" => orbitdata.angularvelocity
    )

    return orbit
end

end
