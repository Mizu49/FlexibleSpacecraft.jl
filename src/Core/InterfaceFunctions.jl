# supporting functions for module `SimulationCore`

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
    orbit = initorbitdata(datanum, orbitinfo)

    # initialize simulation data container
    tl = SimData(time, datanum, attitude, appendages, orbit)

    return tl
end

"""
    _calculate_orbit!

calculate the states of the orbital dynamics of spacecraft
"""
function _calculate_orbit!(
    orbitinfo::Union{OrbitInfo, Nothing},
    currenttime::Real)

    (C_ECI2LVLH, orbit_angularvelocity, orbit_angularposition) = OrbitBase.update_orbitstate!(orbitinfo, currenttime)

    return (C_ECI2LVLH, orbit_angularvelocity, orbit_angularposition)
end

"""
    _calculate_attitude_state!

calculate the states of the attitude dynamics and kinematics
"""
function _calculate_attitude_state!(attitudemodel::AbstractAttitudeDynamicsModel, attidata::AttitudeData, simcnt::Integer, C_ECI2LVLH::SMatrix{3, 3})

    # obtain current quaternion
    quaternion = attidata.quaternion[simcnt]

    # Update current attitude
    C_ECI2Body = ECI2BodyFrame(quaternion)
    attidata.bodyframe[simcnt] = C_ECI2Body * UnitFrame

    # update the roll-pitch-yaw representations
    C_LVLH2BRF = C_ECI2Body * transpose(C_ECI2LVLH)

    # euler angle from LVLH to Body frame is the roll-pitch-yaw angle of the spacecraft
    RPYangle = dcm2euler(C_LVLH2BRF)
    attidata.eulerangle[simcnt] = RPYangle
    # RPYframe representation can be obtained from the LVLH unit frame
    attidata.RPYframe[simcnt] = C_LVLH2BRF * LVLHUnitFrame

    # calculate angular momentum
    attidata.angularmomentum[simcnt] = calc_angular_momentum(attitudemodel, attidata.angularvelocity[simcnt])

    return (C_ECI2Body, C_LVLH2BRF, RPYangle)
end

"""
    _calculate_attitude_disturbance

calculate the disturbance input torque for the attitude dynamics
"""
function _calculate_attitude_disturbance(
    simconfig::SimulationConfig,
    attidistinfo::AttitudeDisturbanceInfo,
    currenttime::Real,
    attitudemodel::AbstractAttitudeDynamicsModel,
    orbit_angularvelocity::Real,
    LVLHframe::Frame,
    C_ECI2Body::SMatrix{3, 3}
    )::SVector{3, Float64}

    # disturbance input calculation
    distinput = AttitudeDisturbance.calc_attitudedisturbance(attidistinfo, attitudemodel.inertia, currenttime, orbit_angularvelocity, C_ECI2Body, SMatrix{3,3}(zeros(3,3)), LVLHframe, simconfig.samplingtime)

    # apply transformation matrix
    distinput = C_ECI2Body * distinput

    return distinput
end

"""
    _calculate_attitude_control

calculate the attitude control input torque
"""
function _calculate_attitude_control(
    controller::AbstractAttitudeController,
    currentRPYangle::SVector{3, Float64},
    targetRPYangle::SVector{3, Float64},
    C_ECI2BRF::SMatrix{3, 3, Float64}
    )::SVector{3, Float64}

    input = transpose(C_ECI2BRF) * control_input!(controller, currentRPYangle, targetRPYangle)

    return input
end

"""
    _calculate_flexible_appendages!

calculate the state of the flexible appendages.

This function is the interface to the flexible appendages simulation
"""
function _calculate_flexible_appendages!(
    appendageinfo::StructuresBase.AppendageInfo,
    datacontainer::Union{AppendageData, Nothing},
    currenttime::Real,
    simcnt::Integer
    )::Tuple

    # check if the flexible appendages exist
    if isnothing(appendageinfo.model)
        # return nothing if flexible appendages don't exist
        distinput = nothing
        ctrlinput = nothing
    else
        # obtain state of the flexible appendages
        datacontainer.physicalstate[simcnt] = modalstate2physicalstate(appendageinfo.model, appendageinfo.internals.currentstate)

        # disturbance input
        distinput = calcstrdisturbance(appendageinfo.disturbance, currenttime)

        # controller of the flexible appendages (for future development)
        ctrlinput = 0

        # data log
        datacontainer.controlinput[simcnt] = ctrlinput
        datacontainer.disturbance[simcnt] = distinput
    end

    return (distinput, ctrlinput)
end

"""
    _calculate_coupling_input

calculate the coupling term of the attitude dynamics and structural dynamics
"""
function _calculate_coupling_input(
    strmodel::Union{AbstractAppendageModel, Nothing},
    strinternals::Union{AppendageInternals, Nothing},
    attitudedata::AttitudeData,
    simcnt::Integer
    )::Tuple

    # calculation of the structural response input for the attitude dynamics
    if isnothing(strinternals)
        # no simulation for flexible appendages
        str_accel = nothing
        str_velocity = nothing
    else
        str_accel    = SVector{2}(strinternals.currentaccel)
        str_velocity = SVector{2}(strinternals.currentstate[(strmodel.DOF+1):end])
    end

    # attitude dynamics
    attitude_angular_velocity = attitudedata.angularvelocity[simcnt]

    # create named tuple
    structure2attitude = (accel = str_accel, velocity = str_velocity)
    attitude2structure = (angularvelocity = attitude_angular_velocity,)

    return (structure2attitude, attitude2structure)
end
