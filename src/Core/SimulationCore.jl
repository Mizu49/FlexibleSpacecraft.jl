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

    # transformation matrix from ECI frame to orbital plane frame
    C_ECI2OrbitPlane = Orbit.ECI2OrbitalPlaneFrame(orbitinfo.orbitalelement)

    # initialize orbit state data array
    orbitdata = Orbit.initorbitdata(datanum, orbitinfo.planeframe)

    # main loop of the simulation
    for cnt = 0:datanum - 1

        # Update orbit state
        orbitdata.angularvelocity[cnt+1] = Orbit.get_angular_velocity(orbitinfo.orbitmodel)
        orbitdata.angularposition[cnt+1] = orbitdata.angularvelocity[cnt+1] * time[cnt+1]

        # update transformation matrix from orbit plane frame to radial along tract frame
        C_OrbitPlane2RAT = Orbit.OrbitalPlaneFrame2RadialAlongTrack(orbitinfo.orbitalelement, orbitdata.angularvelocity[cnt+1], time[cnt+1])
        C_ECI2RAT = C_OrbitPlane2RAT * C_ECI2OrbitPlane

        # calculates transformation matrix from orbital plane frame to radial along frame
        C_ECI2LVLH = T_RAT2LVLH * C_ECI2RAT
        orbitdata.LVLH[cnt + 1] = C_ECI2LVLH * RefFrame

        # Update current attitude
        C_ECI2Body = ECI2BodyFrame(attitudedata.quaternion[cnt+1])
        attitudedata.bodyframe[cnt+1] = C_ECI2Body * RefFrame

        C_LVLH2Body = T_LVLHref2rollpitchyaw * C_ECI2Body * transpose(C_ECI2LVLH)
        attitudedata.RPYframe[cnt+1] = C_LVLH2Body * RefFrame
        attitudedata.eulerangle[cnt+1] = dcm2euler(C_LVLH2Body)

        # Disturbance torque
        disturbance = disturbanceinput(distconfig, attitudemodel.inertia, orbitdata.angularvelocity[cnt+1], C_ECI2Body, C_ECI2LVLH, orbitdata.LVLH[cnt + 1].z)

        # structural input is zero at this point
        structuralinput = zeros(6)

        # Time evolution of the system
        if cnt != datanum - 1

            # Update angular velocity
            attitudedata.angularvelocity[cnt+2] = update_angularvelocity(attitudemodel, time[cnt + 1], attitudedata.angularvelocity[cnt+1], simconfig.samplingtime, attitudedata.bodyframe[cnt+1], disturbance, structuralinput)

            # Update quaternion
            attitudedata.quaternion[cnt+2] = update_quaternion(attitudedata.angularvelocity[cnt+1], attitudedata.quaternion[cnt+1], simconfig.samplingtime)

        end

    end

    return (time, attitudedata, orbitdata)
end

end
