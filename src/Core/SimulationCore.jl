"""
    SimulationCore

submodule contains the high-level interface functions and core implementation of the simulation features
"""
module SimulationCore

using LinearAlgebra, StaticArrays, ProgressMeter
using ..UtilitiesBase, ..Frames, ..OrbitBase, ..AttitudeDisturbance, ..DynamicsBase, ..KinematicsBase, ..StructuresBase, ..StructureDisturbance, ..ParameterSettingBase, ..AttitudeControlBase

export SimData, runsimulation

# function for providing the interface for the simulation feature
include("InterfaceFunctions.jl")

"""
    SimData

data container for one simulation result.

# Fields

* `time::StepRangeLen`: time information
* `datanum::Unsigned`: numbers of simulation data
* `attitude::AttitudeData`: data container for attitude dynamics
* `appendages::AppendageData`: data container for the appendages
* `orbit::OrbitData`: data contaier for orbit data

"""
struct SimData
    time::StepRangeLen
    datanum::Unsigned
    attitude::AttitudeData
    attitude_disturbance::AttitudeDisturbanceData
    appendages::Union{AppendageData, Nothing}
    orbit::Union{OrbitData, Nothing}
end

# main functions for simulation
"""
    runsimulation

Function that runs simulation of the spacecraft attitude-structure coupling problem

# Arguments

* `attitudemodel::AbstractAttitudeDynamicsModel`: dynamics model for the attitude motion
* `initvalue::InitKinematicsData`: struct of initial values for the simulation
* `orbitinfo::OrbitInfo`: model and configuration for the orbital motion
* `orbitinternals::OrbitInternals`: internals of the orbital model
* `distconfig::DisturbanceConfig`: disturbance configuration for the attitude dynamics
* `distinternals::Union{DisturbanceInternals, Nothing}`: internals of the disturbance calculation
*
* `simconfig::SimulationConfig`: configuration for the overall simulation
* `attitude_controller::AbstractAttitudeController: configuration of the attitude controller

# Return value

return value is the instance of `SimData`

# Usage

```julia
simdata = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, orbitinternals, distconfig, distinternals, strdistconfig, strinternals, simconfig, attitudecontroller)
```

"""
@inline function runsimulation(
    attitudemodel::AbstractAttitudeDynamicsModel,
    initvalue::InitKinematicsData,
    orbitinfo::Union{OrbitInfo, Nothing},
    attidistinfo::AttitudeDisturbanceInfo,
    appendageinfo::Union{AppendageInfo, Nothing},
    simconfig::SimulationConfig,
    attitude_controller::AbstractAttitudeController
    )::SimData

    ##### Constants
    # Sampling tl.time
    Ts = simconfig.samplingtime

    # Data containers
    tl = _init_datacontainers(simconfig, initvalue, appendageinfo, orbitinfo, attidistinfo)

    ### main loop of the simulation
    progress = Progress(tl.datanum, 1, "Running...", 20)   # progress meter
    for simcnt = 1:tl.datanum

        # variables
        currenttime = tl.time[simcnt]

        ### orbit state
        C_ECI2LVLH = _calculate_orbit!(orbitinfo, tl.orbit, simcnt, currenttime)

        ### attitude state
        (C_ECI2Body, C_LVLH2BRF, RPYangle) = _calculate_attitude_state!(attitudemodel, tl.attitude, simcnt, C_ECI2LVLH)

        ### input to the attitude dynamics
        # disturbance input
        attitude_disturbance_input = _calculate_attitude_disturbance(simconfig, attidistinfo, tl.attitude_disturbance, simcnt, currenttime, attitudemodel, orbitinfo, C_ECI2LVLH, C_ECI2Body)

        # control input
        attitude_control_input = _calculate_attitude_control(attitude_controller, RPYangle, SVector{3}(zeros(3)), C_ECI2Body)

        ### flexible appendages
        if !isnothing(appendageinfo)
            # calculation of states of the flexible appendages
            _calculate_appendages_state!(appendageinfo, tl.appendages, currenttime, simcnt)
            # calculation of inputs for the flexible appendages
            (structure_disturbance_input, structure_control_input) = _calculate_appendages_input!(appendageinfo, tl.appendages, currenttime, simcnt)
        else
            structure_disturbance_input = nothing
            structure_control_input = nothing
        end

        ### formulate coupling terms between attitude dynamics and structural dynamics
        (structure2attitude, attitude2structure) = _calculate_coupling_input(appendageinfo, tl.attitude, simcnt)

        ### Time evolution of the system
        if simcnt != tl.datanum

            # Update angular velocity
            tl.attitude.angularvelocity[simcnt+1] = update_angularvelocity(attitudemodel, currenttime, tl.attitude.angularvelocity[simcnt], Ts, attitude_disturbance_input, attitude_control_input, structure2attitude.accel, structure2attitude.velocity)

            # Update quaternion
            tl.attitude.quaternion[simcnt+1] = update_quaternion(tl.attitude.angularvelocity[simcnt], tl.attitude.quaternion[simcnt], Ts)

            # Update the state of the flexible appendages
            if !isnothing(appendageinfo)
                tl.appendages.state[simcnt+1] = update_appendages!(appendageinfo, Ts, currenttime, tl.appendages.state[simcnt], attitude2structure.angularvelocity, structure_control_input, structure_disturbance_input)
            end
        end

        # update the progress meter
        next!(progress)
    end

    # return simulation data
    return tl
end

end
