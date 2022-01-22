"""
    runsimulation(model, ECI_frame::Frame, initvalue::TimeLine.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

Function that runs simulation of the spacecraft attitude-structure coupling problem

## Arguments
* `model`: Dynamics model of the system
* `ECI_frame::Frame`: ECI frame of the system
* `initvalue::InitData`: Inital value of the simulation physical states
* `distconfig::DisturbanceConfig`: Disturbanve torque input configuration
* `simconfig::SimulationConfig`: Simulation configuration ParameterSetting

## Return

Return is tuple of `(time, simdata, orbitdata)`

* `time`: 1-D array of the time
* `simdata`: StructArray of trajectory of the physical amount states of the spacecraft system
* `orbitdata`: StructArray of the orbit state trajectory

## Usage
```
(time, simdata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simconfig)
```
"""
function runsimulation(model, ECI_frame::Frame, initvalue::TimeLine.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple

    time = 0:simconfig.samplingtime:simconfig.simulationtime

    # Numbers of simulation data
    datanum = floor(Int, simconfig.simulationtime/simconfig.samplingtime) + 1;

    # Initialize data array
    simdata = initsimulationdata(datanum, initvalue)

    # initialize orbit state data array
    orbitdata = Orbit.initorbitdata(datanum, orbitinfo.planeframe)
    RATframe = initframes(datanum, orbitinfo.planeframe)

    for loopCounter = 0:datanum - 1

        # Update orbit state
        orbitdata.angularvelocity[loopCounter+1] = Orbit.get_angular_velocity(orbitinfo.orbitmodel)
        orbitdata.angularposition[loopCounter+1] = orbitdata.angularvelocity[loopCounter+1] * time[loopCounter+1]

        # update transformation matrix from orbit plane frame to radial along tract frame
        C_RAT = Orbit.OrbitalPlaneFrame2RadialAlongTrack(orbitinfo.orbitalelement, orbitdata.angularvelocity[loopCounter+1], time[loopCounter+1])

        # calculates transformation matrix from orbital plane frame to radial along frame
        C_ECI2LVLH = Orbit.OrbitalPlaneFrame2LVLH(C_RAT)
        orbitdata.LVLH[loopCounter + 1] = C_ECI2LVLH * ECI_frame
        # Update spacecraft Radial Along Track (RAT) frame
        RATframe[loopCounter+1] = Orbit.update_radial_along_track(orbitinfo.planeframe, orbitinfo.orbitalelement, time[loopCounter+1], orbitdata.angularvelocity[loopCounter+1])


        # Update current attitude
        C_ECI2Body = ECI2BodyFrame(simdata.quaternion[loopCounter+1])
        simdata.bodyframe[loopCounter+1] = C_ECI2Body * ECI_frame
        simdata.rollpitchyaw[loopCounter+1] = C_ECI2LVLH * simdata.bodyframe[loopCounter+1]

        # Disturbance torque
        # disturbance = disturbanceinput(distconfig, model.inertia, orbitdata.angularvelocity[loopCounter+1], C_ECI2Body, C_ECI2LVLH, orbitdata.LVLH[loopCounter + 1].z)
        disturbance = transpose(C_ECI2Body) * [0.05, 0, 0]

        # Time evolution of the system
        if loopCounter != datanum - 1

            # Update angular velocity
            simdata.angularvelocity[loopCounter+2] = update_angularvelocity(model, time[loopCounter + 1], simdata.angularvelocity[loopCounter+1], simconfig.samplingtime, simdata.bodyframe[loopCounter+1], disturbance)

            # Update quaternion
            simdata.quaternion[loopCounter+2] = update_quaternion(simdata.angularvelocity[loopCounter+1], simdata.quaternion[loopCounter+1], simconfig.samplingtime)

        end

    end

    return (time, simdata, orbitdata)
end
