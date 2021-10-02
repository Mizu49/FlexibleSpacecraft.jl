"""
    function runsimulation
"""
function runsimulation(model, ECI_frame::Frame, initvalue::TimeLine.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simulation_time::Real, Tsampling::Real)::Tuple

    # Numbers of simulation data
    datanum = floor(Int, simulation_time/Tsampling) + 1;

    # Initialize data array
    simdata = TimeLine.DataTimeLine(initvalue, Tsampling, datanum)

    # initialize orbit state data array
    orbitdata = Orbit.initorbitdata(datanum, orbitinfo.planeframe)
    RATframe = TimeLine.initframes(datanum, orbitinfo.planeframe)

    for loopCounter = 0:datanum - 1

        # Update current attitude
        C = ECI2BodyFrame(simdata.quaternion[:, loopCounter + 1])
        simdata.bodyframes[loopCounter + 1] = C * ECI_frame

        # Update orbit state
        orbitdata[loopCounter+1] = Orbit.updateorbitstate(orbitinfo.orbitalelement, orbitinfo.orbitmodel, simdata.time[loopCounter + 1])

        # Update spacecraft Radial Along Track (RAT) frame
        RATframe[loopCounter+1] = Orbit.update_radial_along_track(orbitinfo.planeframe, orbitinfo.orbitalelement, simdata.time[loopCounter+1], orbitdata.angularvelocity[loopCounter+1])

        # Disturbance torque
        disturbance = disturbanceinput(distconfig)

        # Time evolution of the system
        if loopCounter != datanum - 1

            # Update angular velocity
            simdata.angularvelocity[:, loopCounter + 2] = calc_angular_velocity(model, simdata.time[loopCounter + 1], simdata.angularvelocity[:, loopCounter + 1], Tsampling, simdata.bodyframes[loopCounter + 1], disturbance)

            # Update quaternion
            simdata.quaternion[:, loopCounter + 2] = calc_quaternion(simdata.angularvelocity[:,loopCounter + 1], simdata.quaternion[:, loopCounter + 1], Tsampling)

        end

    end

    return (simdata, orbitdata)
end
