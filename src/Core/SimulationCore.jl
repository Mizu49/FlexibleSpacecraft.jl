"""
    SimulationCore

submodule contains the high-level interface functions and core implementation of the simulation features
"""
module SimulationCore

using LinearAlgebra, StaticArrays, ProgressMeter
using ..UtilitiesBase, ..Frames, ..OrbitBase, ..AttitudeDisturbance, ..DynamicsBase, ..KinematicsBase, ..StructuresBase, ..StructureDisturbance, ..ParameterSettingBase, ..AttitudeControlBase

export SimData, runsimulation

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
    appendages::Union{AppendageData, Nothing}
    orbit::OrbitData
end


############# runsimulation function ##################################
"""
    runsimulation

Function that runs simulation of the spacecraft attitude-structure coupling problem

# Arguments

* `attitudemodel::AbstractAttitudeDynamicsModel`: dynamics model for the attitude motion
* `strmodel::AbstractStructuresModel`: dynamics model for the flexible appendages motion
* `initvalue::InitKinematicsData`: struct of initial values for the simulation
* `orbitinfo::OrbitInfo`: model and configuration for the orbital motion
* `orbitinternals::OrbitInternals`: internals of the orbital model
* `distconfig::DisturbanceConfig`: disturbance configuration for the attitude dynamics
* `distinternals::Union{DisturbanceInternals, Nothing}`: internals of the disturbance calculation
* `strdistconfig::AbstractStrDistConfig`: disturbance configuration for the structural dynamics
* `strinternals::Union{AppendageInternals, Nothing}`: internals for the structural dynamics simulation
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
    strmodel::AbstractStructuresModel,
    initvalue::InitKinematicsData,
    orbitinfo::OrbitInfo,
    orbitinternals::OrbitInternals,
    distconfig::DisturbanceConfig,
    distinternals::Union{DisturbanceInternals, Nothing},
    strdistconfig::AbstractStrDistConfig,
    strinternals::Union{AppendageInternals, Nothing},
    simconfig::SimulationConfig,
    attitude_controller::AbstractAttitudeController
    )::SimData

    ##### Constants
    # Sampling tl.time
    Ts = simconfig.samplingtime

    # Data containers
    tl = _init_datacontainers(simconfig, initvalue, strmodel, orbitinfo)

    ### main loop of the simulation
    progress = Progress(tl.datanum, 1, "Simulation running...", 50)   # progress meter
    for simcnt = 1:tl.datanum

        # variables
        currenttime = tl.time[simcnt]

        ### orbit state
        (C_ECI2LVLH, orbit_angularvelocity, orbit_angularposition) = _calculate_orbit_state(orbitinfo, orbitinternals, currenttime)

        ### attitude state
        # Update current attitude
        C_ECI2Body = ECI2BodyFrame(tl.attitude.quaternion[simcnt])
        tl.attitude.bodyframe[simcnt] = C_ECI2Body * UnitFrame

        # update the roll-pitch-yaw representations
        C_LVLH2BRF = C_ECI2Body * transpose(C_ECI2LVLH)
        # euler angle from LVLH to Body frame is the roll-pitch-yaw angle of the spacecraft
        RPYangle = dcm2euler(C_LVLH2BRF)
        tl.attitude.eulerangle[simcnt] = RPYangle
        # RPYframe representation can be obtained from the LVLH unit frame
        tl.attitude.RPYframe[simcnt] = C_LVLH2BRF * LVLHUnitFrame

        # calculate angular momentum
        tl.attitude.angularmomentum[simcnt] = calc_angular_momentum(attitudemodel, tl.attitude.angularvelocity[simcnt])

        ### input to the attitude dynamics
        # disturbance input
        attitude_disturbance_input = transpose(C_ECI2Body) * calc_attitudedisturbance(distconfig, distinternals, attitudemodel.inertia, currenttime, tl.orbit.angularvelocity[simcnt], C_ECI2Body, zeros(3,3), tl.orbit.LVLH[simcnt].z, Ts)
        attitude_disturbance_input = SVector{3}(attitude_disturbance_input)

        # control input
        attitude_control_input = transpose(C_ECI2Body) * control_input!(attitude_controller, RPYangle, SVector{3}([0.0, 0.0, 0.0]))

        ### flexible appendages state
        if !isnothing(strmodel)
            tl.appendages.physicalstate[simcnt] = modalstate2physicalstate(strmodel, strinternals.currentstate)
        end

        ### input to the structural dynamics
        if isnothing(strmodel)
            structure_disturbance_input = nothing
            structure_control_input = nothing
        else
            # disturbance input
            structure_disturbance_input = calcstrdisturbance(strdistconfig, currenttime)
            # control input
            structure_control_input = 0
            # data log
            tl.appendages.controlinput[simcnt] = structure_control_input
            tl.appendages.disturbance[simcnt] = structure_disturbance_input
        end

        ### attitude-structure coupling dynamics
        # calculation of the structural response input for the attitude dynamics
        if isnothing(strinternals)
            # no simulation for flexible appendages
            coupling_structure_accel = nothing
            coupling_structure_velocity = nothing
        else
            coupling_structure_accel    = SVector{2}(strinternals.currentaccel)
            coupling_structure_velocity = SVector{2}(strinternals.currentstate[(strmodel.DOF+1):end])
        end
        # attitude dynamics
        coupling_angular_velocity = tl.attitude.angularvelocity[simcnt]

        ### Time evolution of the system
        if simcnt != tl.datanum

            # Update angular velocity
            tl.attitude.angularvelocity[simcnt+1] = update_angularvelocity(attitudemodel, currenttime, tl.attitude.angularvelocity[simcnt], Ts, attitude_disturbance_input, attitude_control_input, coupling_structure_accel, coupling_structure_velocity)

            # Update quaternion
            tl.attitude.quaternion[simcnt+1] = update_quaternion(tl.attitude.angularvelocity[simcnt], tl.attitude.quaternion[simcnt], Ts)

            # Update the state of the flexible appendages
            if !isnothing(strmodel)
                tl.appendages.state[simcnt+1] = update_strstate!(strmodel, strinternals, Ts, currenttime, tl.appendages.state[simcnt], coupling_angular_velocity, structure_control_input, structure_disturbance_input)
            end
        end

        # update the progress meter
        next!(progress)
    end

    # return simulation data
    return tl
end

### supporting functions
"""
    _init_datacontainers

initialize data container
"""
function _init_datacontainers(simconfig, initvalue, strmodel, orbitinfo)

    time = 0:simconfig.samplingtime:simconfig.simulationtime

    # Numbers of simulation data
    datanum = floor(Int, simconfig.simulationtime/simconfig.samplingtime) + 1;

    # Initialize data containers for the attitude dynamics
    attitude = initattitudedata(datanum, initvalue)

    # initialize data container for the structural motion of the flexible appendages
    appendages = initappendagedata(strmodel, [0, 0, 0, 0], datanum)

    # initialize orbit state data array
    orbit = initorbitdata(datanum, orbitinfo.planeframe)

    # initialize simulation data container
    tl = SimData(time, datanum, attitude, appendages, orbit)

    return tl
end

"""
    _calculate_orbit_state

calculate the states of the orbital dynamics of spacecraft
"""
function _calculate_orbit_state(orbitinfo::OrbitInfo, orbitinternals::OrbitInternals, currenttime::Real)

    (orbit_angularvelocity, orbit_angularposition) = update_orbitstate!(orbitinfo, orbitinternals, currenttime)

    # calculation of transformation matrix of the LVLH frame
    C_ECI2LVLH = ECI2ORF(orbitinfo.orbitalelement, orbit_angularposition)

    return (C_ECI2LVLH, orbit_angularvelocity, orbit_angularposition)
end

end
