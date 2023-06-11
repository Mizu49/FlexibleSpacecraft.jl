# simulation parameter setting script

spacecraft = Dict()

## simulation configuration
spacecraft["config"] = Dict(
    "name" => "example spacecraft",
    "note" => "This model is for test purpose",
    "sampling time" => 1e-3,
    "time length" => 100
)

## initlal value of the attitude
spacecraft["initial value"] = Dict(
    "roll-pitch-yaw" => [0.0, 0.0, 0.0],
    "angular velocity" => [0.0, 0.0, 0.0]
)

## spacecraft attitdue dynamics model configuration
spacecraft["attitude dynamics"] = Dict(
    "model" => "constrained modeling",
    "inertia" => [
        1000.0    0.0    0.0
        0.0    1000.0    0.0
        0.0       0.0 1000.0
    ],
    # flexible appendages
    "appendages" => Dict(
        "modeling" => "spring-mass",
        "system" => Dict(
            "coord" => "physical",
            "DOF" => 2,
            "mass" => [
                100  0
                0   50
            ],
            "stiffness" => [
                6e4  -1e4
                -1e4  1e4
            ],
            "damping" => Dict(
                "config" => "Rayleigh",
                "alpha" => 0,
                "beta" => 0.005
            ),
            "control input" => Dict(
                "dimension" => 1,
                "coefficient" => [
                    5
                    0
                ]
            ),
            "disturbance input" => Dict(
                "dimension" => 1,
                "coefficient" => [
                    3
                    0
                ]
            )
        ),
    )
)

## attitude disturbance configuration
spacecraft["disturbance"] = Dict(
    "constant torque" => [0.0, 0.0, 0.0],
    "gravitational torque" => false,
    "step trajectory" => Dict(
        "value" => [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "endurance" => [1, 300]
    )
)

## Orbital dynamics
spacecraft["Orbit"] = Dict(
    # Specify the model used in the simulation of orbital motion
    "Dynamics model" => "none",
    # Orbital elements
    "Orbital elements" => Dict(
        "right ascension" => 0.0,
        "inclination" => 0.0,
        "semimajor axis" => 6371e3,
        "eccentricity" => 0.0,
        "argument of perigee" => 0.0,
        "true anomaly at epoch" => 0.0
    ),
    # Note for this parameter setting
    "OrbitInfo" => "Test orbit parameters"
)

## Attitude control
spacecraft["attitude controller"] = Dict(
    "strategy" => "PID",
    "config" => Dict(
        "Pgain" => [
            100 0 0
            0 100 0
            0 0 100
        ],
        "Igain" => [
            0 0 0
            0 0 0
            0 0 0
        ],
        "Dgain" => [
            0 0 0
            0 0 0
            0 0 0
        ]
    )
)
