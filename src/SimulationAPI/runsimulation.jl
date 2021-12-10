"""
    function runsimulation
"""
function runsimulation(model, ECI_frame::Frame, initvalue::TimeLine.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simulation_time::Real, Tsampling::Real)::Tuple

    time = 0:Tsampling:simulation_time

    # Numbers of simulation data
    datanum = floor(Int, simulation_time/Tsampling) + 1;

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

        # Disturbance torque
        disturbance = disturbanceinput(distconfig, model.inertia, orbitdata.angularvelocity[loopCounter+1], C_ECI2Body, C_ECI2LVLH, orbitdata.LVLH[loopCounter + 1].z)

        # Time evolution of the system
        if loopCounter != datanum - 1

            # Update angular velocity
            simdata.angularvelocity[loopCounter+2] = calc_angular_velocity(model, time[loopCounter + 1], simdata.angularvelocity[loopCounter+1], Tsampling, simdata.bodyframe[loopCounter+1], disturbance)

            # Update quaternion
            simdata.quaternion[loopCounter+2] = calc_quaternion(simdata.angularvelocity[loopCounter+1], simdata.quaternion[loopCounter+1], Tsampling)

        end

    end

    return (time, simdata, orbitdata)
end
