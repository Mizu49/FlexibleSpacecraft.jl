"""
    SimulationCore

submodule contains the high-level interface functions and core implementation of the simulation features
"""
module SimulationCore

using LinearAlgebra, StaticArrays, ProgressMeter
using ..Frames, ..OrbitBase, ..AttitudeDisturbance, ..DynamicsBase, ..KinematicsBase, ..StructuresBase, ..StructureDisturbance, ..ParameterSettingBase, ..AttitudeControlBase

export runsimulation

############# runsimulation function ##################################
"""
    runsimulation(attitudemodel, initvalue::KinematicsBase.InitData, orbitinfo::OrbitBase.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

Function that runs simulation of the spacecraft attitude-structure coupling problem

# Arguments

* `attitudemodel`: KinematicsBase dynamics model of the system
* `strmodel`: Structural model of the flexible appendages
* `initvalue::InitData`: Inital value of the simulation physical states
* `orbitinfo::OrbitBase.OrbitInfo`: information and model definition of the orbital dynamics
* `distconfig::DisturbanceConfig`: Disturbanve torque input configuration
* `simconfig::SimulationConfig`: Simulation configuration ParameterSettingBase

# Return value

Return is tuple of `(time, attidata, orbitdata, strdata``)`

* `time`: 1-D array of the time
* `attidata`: Struct of time trajectory of the physical amount states of the spacecraft system
* `orbitdata`: Struct of the orbit state trajectory
* `strdata`: Struct of the trajectory of the state of the flexible appendage

# Usage

```julia
(time, attitudedata, orbitdata, strdata) = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, simconfig)
```

"""
@inline function runsimulation(
    attitudemodel,
    strmodel,
    initvalue::KinematicsBase.InitData,
    orbitinfo::OrbitBase.OrbitInfo,
    distconfig::DisturbanceConfig,
    strdistconfig::AbstractStrDistConfig,
    simconfig::SimulationConfig,
    attitude_controller
    )::Tuple

    ##### Constants
    # Sampling time
    Ts = simconfig.samplingtime

    # transformation matrix from ECI frame to orbital plane frame
    C_ECI2OrbitPlane = OrbitBase.ECI2OrbitalPlaneFrame(orbitinfo.orbitalelement)

    # Data containers
    (datanum, time, attidata, strdata, orbitdata) = _init_datacontainers(simconfig, initvalue, strmodel, orbitinfo)

    ##### main loop of the simulation
    prog = Progress(datanum, 1, "Simulation running...", 50)   # progress meter
    for iter = 1:datanum

        ############### orbit state ##################################################
        orbitdata.angularvelocity[iter] = get_angular_velocity(orbitinfo.orbitmodel)
        orbitdata.angularposition[iter] = orbitdata.angularvelocity[iter] * time[iter]

        # calculation of the LVLH frame and its transformation matrix
        C_OrbitPlane2RAT = OrbitalPlaneFrame2RadialAlongTrack(orbitinfo.orbitalelement, orbitdata.angularvelocity[iter], time[iter])
        C_ECI2RAT = C_OrbitPlane2RAT * C_ECI2OrbitPlane
        C_ECI2LVLH = T_RAT2LVLH * C_ECI2RAT

        ############### attitude #####################################################
        # Update current attitude
        C_ECI2Body = ECI2BodyFrame(attidata.quaternion[iter])
        attidata.bodyframe[iter] = C_ECI2Body * UnitFrame

        # update the roll-pitch-yaw representations
        C_RAT2Body = C_ECI2Body * transpose(C_ECI2RAT)
        C_LVLH2Body = T_RAT2LVLH * C_ECI2Body * transpose(C_ECI2LVLH)
        # euler angle from RAT to Body frame is the roll-pitch-yaw angle of the spacecraft
        current_RPY = dcm2euler(C_LVLH2Body)
        attidata.eulerangle[iter] = current_RPY
        # RPYframe representation can be obtained from the LVLH unit frame
        attidata.RPYframe[iter] = C_LVLH2Body * LVLHUnitFrame

        ############### flexible appendages state ####################################
        strdata.physicalstate[iter] = modalstate2physicalstate(strmodel, strdata.state[iter])

        ############### control and disturbance torque input to the attitude dynamics ############
        # attidistinput = disturbanceinput(distconfig, attitudemodel.inertia, orbitdata.angularvelocity[iter], C_ECI2Body, C_ECI2RAT, orbitdata.LVLH[iter].z)
        attidistinput = transpose(C_ECI2Body) * [1, 0, 0]

        attictrlinput = transpose(C_ECI2Body) * control_input!(attitude_controller, current_RPY, [0, 0, 0])

        ############### control and disturbance input to the flexible appendages
        strdistinput = calcstrdisturbance(strdistconfig, time[iter])
        strctrlinput = 0

        strdata.controlinput[iter] = strctrlinput
        strdata.disturbance[iter] = strdistinput


        ############### coupling dynamics of the flexible spacecraft ###############

        # calculation of the structural response input for the attitude dynamics
        currentstrstate = strdata.state[iter]
        if iter == 1
            straccel = currentstrstate[(strmodel.DOF+1):end] / Ts
        else
            previousstrstate = strdata.state[iter-1]
            straccel = (currentstrstate[(strmodel.DOF+1):end] - previousstrstate[(strmodel.DOF+1):end]) / Ts
        end
        strvelocity = currentstrstate[(strmodel.DOF+1):end]

        # angular velocity of the attitude dynamics for the structural coupling input
        attiinput = attidata.angularvelocity[iter]

        ################## Time evolution of the system ##############################
        if iter != datanum

            # Update angular velocity
            attidata.angularvelocity[iter+1] = update_angularvelocity(attitudemodel, time[iter], attidata.angularvelocity[iter], Ts, attidata.bodyframe[iter], attidistinput, attictrlinput, straccel, strvelocity)

            # Update quaternion
            attidata.quaternion[iter+1] = update_quaternion(attidata.angularvelocity[iter], attidata.quaternion[iter], Ts)

            # Update the state of the flexible appendages
            strdata.state[iter+1] = updatestate(strmodel, Ts, time[iter], strdata.state[iter], attiinput, strctrlinput, strdistinput)

        end

        next!(prog) # update the progress meter
    end

    return (time, attidata, orbitdata, strdata)
end

function _init_datacontainers(simconfig, initvalue, strmodel, orbitinfo)

    time = 0:simconfig.samplingtime:simconfig.simulationtime

    # Numbers of simulation data
    datanum = floor(Int, simconfig.simulationtime/simconfig.samplingtime) + 1;

    # Initialize data containers for the attitude dynamics
    attidata = initattitudedata(datanum, initvalue)

    # initialize data container for the structural motion of the flexible appendages
    strdata = initappendagedata(strmodel, [0, 0, 0, 0], datanum)

    # initialize orbit state data array
    orbitdata = initorbitdata(datanum, orbitinfo.planeframe)

    return (datanum, time, attidata, strdata, orbitdata)
end

end
