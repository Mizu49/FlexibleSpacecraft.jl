"""
    DataAPI

API submodule for handling the `DataFrame` of the simulation results

# Usage

Before using any features provided by `DataAPI`, you need to run simulation with `runsimulation()`

```julia
(time, attitudedata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)
```

Then you get your simulation result as `StructVector`. You might want to convert that to the tabular data structure as `DataFrame` object. The following code will generate `DataFrame` object:

```julia
outputdata = SimData(time, attitudedata, orbitdata)
```

You can export your simulation result as CSV files. Simply run the following code specifying the directory in which you may want to save:

```julia
write("output", outputdata)
```

Here, we specify `output` as an output root directory. The system generates a new directory with time stamp of when the simulation result was generated. The time stamp is with format `yyyy-mm-dd--hh-mm-ss`. Separated files are generated.

* `yyyy-mm-dd--hh-mm-ss--attitude.csv`: CSV file for the attitude tabular data. (time / quaternion / angular velocity / euler angles (roll, pitch, yaw))
* `yyyy-mm-dd--hh-mm-ss--orbit.csv`: CSV file for the orbita tabular data (time, angular position, angular velocity)
"""
module DataAPI

using DataFrames, CSV, StaticArrays, StructArrays, Dates
using ..Frames, ..TimeLine

export SimData

"""
    SimData

struct of the table data frame from simulation results

# Fields

* `timestamp::String`: time stamp of when the data is generated
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

    timestamp::String
    attitude::DataFrame
    orbit::DataFrame

    # Constructor
    SimData(time::AbstractRange{<:Real}, attitudedata::StructVector, orbitdata::StructVector) = begin

        # Get current time stamp in UTC
        timestamp = Dates.format(Dates.now(), "yyyy-mm-dd--HH-MM-SS")

        # create `DataFrame` objects
        attitude = _convert_attitude(time, attitudedata)
        orbit = _convert_orbit(time, orbitdata)

        new(timestamp, attitude, orbit)
    end
end

"""
    Base.write(path::AbstractString, simdata::SimData)

save simulation result `simdata::SimData` as output files in CSV format
"""
function Base.write(path::AbstractString, simdata::SimData)

    print("generating output file...")

    mkdir("$path/$(simdata.timestamp)")

    CSV.write("$path/$(simdata.timestamp)/$(simdata.timestamp)--attitude.csv", simdata.attitude)
    CSV.write("$path/$(simdata.timestamp)/$(simdata.timestamp)--orbit.csv", simdata.orbit)

    print("done!")
    return nothing
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
