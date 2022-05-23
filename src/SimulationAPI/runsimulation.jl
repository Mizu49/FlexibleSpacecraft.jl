"""
    runsimulation(model, initvalue::Attitude.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

Function that runs simulation of the spacecraft attitude-structure coupling problem

## Arguments
* `model`: Dynamics model of the system
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
(time, attitudedata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)
```
"""
function runsimulation(model, initvalue::Attitude.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

    time = 0:simconfig.samplingtime:simconfig.simulationtime

    # Numbers of simulation data
    datanum = floor(Int, simconfig.simulationtime/simconfig.samplingtime) + 1;

    # Initialize data array
    attitudedata = initsimulationdata(datanum, initvalue)

    C_ECI2OrbitPlane = Orbit.ECI2OrbitalPlaneFrame(orbitinfo.orbitalelement)

    # initialize orbit state data array
    orbitdata = Orbit.initorbitdata(datanum, orbitinfo.planeframe)
    RATframe = initframes(datanum, orbitinfo.planeframe)

    for loopCounter = 0:datanum - 1

        # Update orbit state
        orbitdata.angularvelocity[loopCounter+1] = Orbit.get_angular_velocity(orbitinfo.orbitmodel)
        orbitdata.angularposition[loopCounter+1] = orbitdata.angularvelocity[loopCounter+1] * time[loopCounter+1]

        # update transformation matrix from orbit plane frame to radial along tract frame
        C_OrbitPlane2RAT = Orbit.OrbitalPlaneFrame2RadialAlongTrack(orbitinfo.orbitalelement, orbitdata.angularvelocity[loopCounter+1], time[loopCounter+1])
        C_ECI2RAT = C_OrbitPlane2RAT * C_ECI2OrbitPlane

        # calculates transformation matrix from orbital plane frame to radial along frame
        C_ECI2LVLH = T_RAT2LVLH * C_ECI2RAT
        orbitdata.LVLH[loopCounter + 1] = C_ECI2LVLH * RefFrame
        # Update spacecraft Radial Along Track (RAT) frame
        RATframe[loopCounter+1] = Orbit.update_radial_along_track(orbitinfo.planeframe, orbitinfo.orbitalelement, time[loopCounter+1], orbitdata.angularvelocity[loopCounter+1])


        # Update current attitude
        C_ECI2Body = ECI2BodyFrame(attitudedata.quaternion[loopCounter+1])
        attitudedata.bodyframe[loopCounter+1] = C_ECI2Body * RefFrame

        C_LVLH2Body = T_LVLHref2rollpitchyaw * C_ECI2Body * transpose(C_ECI2LVLH)
        attitudedata.RPYframe[loopCounter+1] = C_LVLH2Body * RefFrame
        attitudedata.eulerangle[loopCounter+1] = dcm2euler(C_LVLH2Body)

        # Disturbance torque
        disturbance = disturbanceinput(distconfig, model.inertia, orbitdata.angularvelocity[loopCounter+1], C_ECI2Body, C_ECI2LVLH, orbitdata.LVLH[loopCounter + 1].z)

        # structural input is zero at this point
        structuralinput = zeros(6)

        # Time evolution of the system
        if loopCounter != datanum - 1

            # Update angular velocity
            attitudedata.angularvelocity[loopCounter+2] = update_angularvelocity(model, time[loopCounter + 1], attitudedata.angularvelocity[loopCounter+1], simconfig.samplingtime, attitudedata.bodyframe[loopCounter+1], disturbance, structuralinput)

            # Update quaternion
            attitudedata.quaternion[loopCounter+2] = update_quaternion(attitudedata.angularvelocity[loopCounter+1], attitudedata.quaternion[loopCounter+1], simconfig.samplingtime)

        end

    end

    return (time, attitudedata, orbitdata)
end
