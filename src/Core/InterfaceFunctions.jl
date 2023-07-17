# supporting functions for module `SimulationCore`

"""
    _init_datacontainers

initialize data container
"""
function _init_datacontainers(
    simconfig::SimulationConfig,
    initvalue::InitKinematicsData,
    appendageinfo::Union{AppendageInfo, Nothing},
    orbitinfo::Union{OrbitInfo, Nothing},
    attidistinfo::AttitudeDisturbanceInfo
    )::SimData

    time = 0:simconfig.samplingtime:simconfig.simulationtime

    # Numbers of simulation data
    datanum = floor(Int, simconfig.simulationtime/simconfig.samplingtime) + 1;

    # Initialize data containers for the attitude dynamics
    attitude = initattitudedata(datanum, initvalue)

    # initialize data container for the structural motion of the flexible appendages
    if isnothing(appendageinfo)
        appendages = nothing
    else
        appendages = initappendagedata(appendageinfo, [0, 0, 0, 0], datanum)
    end

    # initialize orbit state data array
    if isnothing(orbitinfo)
        orbit = nothing
    else
        orbit = initorbitdata(datanum, orbitinfo)
    end

    # initialize attitude disturbance
    attidist = init_attitude_disturbance_data(datanum, attidistinfo)

    # initialize simulation data container
    tl = SimData(time, datanum, attitude, attidist, appendages, orbit)

    return tl
end

"""
    _calculate_orbit!

calculate the states of the orbital dynamics of spacecraft
"""
function _calculate_orbit!(
    orbitinfo::OrbitInfo,
    orbitdata::OrbitData,
    simcnt::Integer,
    currenttime::Real
    )::SMatrix{3, 3}

    # call the function in module `OrbitBase`
    C_ECI2LVLH = update_orbitstate!(orbitinfo, currenttime)

    orbitdata.angularposition[simcnt] = orbitinfo.internals.angularposition
    orbitdata.angularvelocity[simcnt] = orbitinfo.internals.angularvelocity
    orbitdata.LVLH[simcnt] = C_ECI2LVLH * UnitFrame

    return C_ECI2LVLH
end

function _calculate_orbit!(
    orbitinfo::Nothing,
    orbitdata::Nothing,
    simcnt::Integer,
    currenttime::Real
    )::SMatrix{3, 3}
    # orbital dynamics is not considered

    # rotation matrix is identity
    C_ECI2LVLH = SMatrix{3, 3}(I)

    return C_ECI2LVLH
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
    attidistdata::AttitudeDisturbanceData,
    simcnt::Unsigned,
    currenttime::Real,
    attitudemodel::AbstractAttitudeDynamicsModel,
    orbitinfo::Union{OrbitInfo, Nothing},
    C_ECI2LVLH::SMatrix{3, 3},
    C_ECI2Body::SMatrix{3, 3}
    )::SVector{3, Float64}

    if isnothing(orbitinfo)
        orbit_angularvelocity = 0.0
    else
        orbit_angularvelocity = orbitinfo.internals.angularvelocity
    end

    # altitude of the spacecraft
    orbital_altitude = 400e3

    # disturbance input calculation
    distinput = calc_attitudedisturbance!(attidistinfo, attitudemodel, attidistdata, simcnt, currenttime, C_ECI2Body, C_ECI2LVLH, orbital_altitude ,simconfig.samplingtime)

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
    appendageinfo::AppendageInfo,
    datacontainer::AppendageData,
    currenttime::Real,
    simcnt::Integer
    )::Tuple

    # obtain state of the flexible appendages
    datacontainer.physicalstate[simcnt] = modalstate2physicalstate(appendageinfo.model, appendageinfo.internals.currentstate)

    # disturbance input
    distinput = calcstrdisturbance(appendageinfo.disturbance, currenttime)

    # controller of the flexible appendages (for future development)
    ctrlinput = 0

    # data log
    datacontainer.controlinput[simcnt] = ctrlinput
    datacontainer.disturbance[simcnt] = distinput

    return (distinput, ctrlinput)
end

function _calculate_flexible_appendages!(
    appendageinfo::Nothing,
    datacontainer::Nothing,
    currenttime::Real,
    simcnt::Integer
    )::Tuple

    distinput = nothing
    ctrlinput = nothing

    return (distinput, ctrlinput)
end

"""
    _calculate_coupling_input

calculate the coupling term of the attitude dynamics and structural dynamics
"""
function _calculate_coupling_input(
    appendageinfo::AppendageInfo,
    attitudedata::AttitudeData,
    simcnt::Integer
    )::Tuple

    # calculation of the structural response input for the attitude dynamics
    str_accel    = SVector{2}(appendageinfo.internals.currentaccel)
    str_velocity = SVector{2}(appendageinfo.internals.currentstate[(appendageinfo.model.DOF+1):end])

    # attitude dynamics
    attitude_angular_velocity = attitudedata.angularvelocity[simcnt]

    # create named tuple
    structure2attitude = (accel = str_accel, velocity = str_velocity)
    attitude2structure = (angularvelocity = attitude_angular_velocity,)

    return (structure2attitude, attitude2structure)
end

function _calculate_coupling_input(
    appendageinfo::Nothing,
    attitudedata::AttitudeData,
    simcnt::Integer
)::Tuple

    # no data available when flexible appendages are not considered
    str_accel    = nothing
    str_velocity = nothing

    # attitude dynamics
    attitude_angular_velocity = attitudedata.angularvelocity[simcnt]

    # create named tuple
    structure2attitude = (accel = str_accel, velocity = str_velocity)
    attitude2structure = (angularvelocity = attitude_angular_velocity,)

    return (structure2attitude, attitude2structure)
end
