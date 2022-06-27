module NoOrbit

export NoOrbitModel

struct NoOrbitModel
    # no member variables
end

function get_angular_velocity(orbitmodel::NoOrbitModel)
    return 0
end

function get_velocity(orbitmodel::NoOrbitModel)
    return 0
end

function get_timeperiod(orbitmodel::NoOrbitModel; unit = "second")
    return 0
end

end
