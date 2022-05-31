"""
    SimulationCore

submodule contains the high-level interface functions and core implementation of the simulation features
"""
module SimulationCore

using LinearAlgebra, StaticArrays
using ..Frames, ..Orbit, ..Disturbance, ..DynamicsBase, ..Attitude, ..Structures, ..ParameterSetting

export runsimulation

############# runsimulation function ##################################
"""
    runsimulation(attitudemodel, initvalue::Attitude.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

Function that runs simulation of the spacecraft attitude-structure coupling problem

## Arguments
* `attitudemodel`: Attitude dynamics model of the system
* `strmodel`: Structural model of the flexible appendages
* `initvalue::InitData`: Inital value of the simulation physical states
* `distconfig::DisturbanceConfig`: Disturbanve torque input configuration
* `simconfig::SimulationConfig`: Simulation configuration ParameterSetting

## Return

Return is tuple of `(time, attitudedata, orbitdata)`

* `time`: 1-D array of the time
* `attitudedata`: StructArray of trajectory of the physical amount states of the spacecraft system
* `orbitdata`: StructArray of the orbit state trajectory

## Usage
```
(time, attitudedata, orbitdata) = runsimulation(attitudemodel, initvalue, orbitinfo, distconfig, simconfig)
```
"""
function runsimulation(attitudemodel, strmodel, initvalue::Attitude.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

    # array of the time
    time = 0:simconfig.samplingtime:simconfig.simulationtime

    # Numbers of simulation data
    datanum = floor(Int, simconfig.simulationtime/simconfig.samplingtime) + 1;

    # Initialize data containers
    attitudedata = initattitudedata(datanum, initvalue)

    # initialize data container for the structural motion of the flexible appendages
    strdata = initappendagedata(strmodel, [0, 0, 0, 0], datanum)

    # transformation matrix from ECI frame to orbital plane frame
    C_ECI2OrbitPlane = Orbit.ECI2OrbitalPlaneFrame(orbitinfo.orbitalelement)

    # initialize orbit state data array
    orbitdata = Orbit.initorbitdata(datanum, orbitinfo.planeframe)

    # main loop of the simulation
    for cnt = 0:datanum - 1

        ############### orbit state ##################################################
        orbitdata.angularvelocity[cnt+1] = Orbit.get_angular_velocity(orbitinfo.orbitmodel)
        orbitdata.angularposition[cnt+1] = orbitdata.angularvelocity[cnt+1] * time[cnt+1]

        # calculation of the LVLH frame and its transformation matrix
        C_OrbitPlane2RAT = Orbit.OrbitalPlaneFrame2RadialAlongTrack(orbitinfo.orbitalelement, orbitdata.angularvelocity[cnt+1], time[cnt+1])
        C_ECI2LVLH = T_RAT2LVLH * C_OrbitPlane2RAT * C_ECI2OrbitPlane
        orbitdata.LVLH[cnt + 1] = C_ECI2LVLH * RefFrame

        ############### attitude #####################################################
        # Update current attitude
        C_ECI2Body = ECI2BodyFrame(attitudedata.quaternion[cnt+1])
        attitudedata.bodyframe[cnt+1] = C_ECI2Body * RefFrame
        # update the euler angle representations
        C_LVLH2Body = T_LVLHref2rollpitchyaw * C_ECI2Body * transpose(C_ECI2LVLH)
        attitudedata.RPYframe[cnt+1] = C_LVLH2Body * RefFrame
        attitudedata.eulerangle[cnt+1] = dcm2euler(C_LVLH2Body)

        ############### flexible appendages state ####################################
        strdata.physicalstate[cnt+1] = modalstate2physicalstate(strmodel, strdata.state[cnt+1])

        ############### disturbance torque input to the attitude dynamics ############
        # Disturbance torque
        disturbance = disturbanceinput(distconfig, attitudemodel.inertia, orbitdata.angularvelocity[cnt+1], C_ECI2Body, C_ECI2LVLH, orbitdata.LVLH[cnt + 1].z)

        ############### control and disturbance input to the flexible appendages
        strdistinput = 5 * sin(10* time[cnt+1])
        strctrlinput = 0

        strdata.controlinput[cnt+1] = strctrlinput
        strdata.disturbance[cnt+1] = strdistinput

        ############### coupling dynamics of the flexible spacecraft ###############
        straccel = zeros(2)
        strvelocity = zeros(2)
        attitudeinput = attitudedata.angularvelocity[cnt+1]

        ################## Time evolution of the system ##############################
        if cnt != datanum - 1

            # Update angular velocity
            attitudedata.angularvelocity[cnt+2] = update_angularvelocity(attitudemodel, time[cnt+1], attitudedata.angularvelocity[cnt+1], simconfig.samplingtime, attitudedata.bodyframe[cnt+1], disturbance, straccel, strvelocity)

            # Update quaternion
            attitudedata.quaternion[cnt+2] = update_quaternion(attitudedata.angularvelocity[cnt+1], attitudedata.quaternion[cnt+1], simconfig.samplingtime)

            # Update the state of the flexible appendages
            strdata.state[cnt+2] = updatestate(strmodel, simconfig.samplingtime, time[cnt+1], strdata.state[cnt+1], attitudeinput, strctrlinput, strdistinput)

        end

    end

    return (time, attitudedata, orbitdata, strdata)
end

end
