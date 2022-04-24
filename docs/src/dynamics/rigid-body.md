# Rigid body dynamics model

Submodule `RigidBody` provides simulation algorithm for the simple rigid body spacecraft attitude dynamics. This model does not include the flexible structural appendages. 

```@docs
RigidBody
```

## Parameter settings for rigid body attitude dynamics

Parameter setting YAML files should be organized as follows:

```yaml
property: dynamics
name: example spacecraft
note: This model is for test purpose

dynamicsmodel: Rigid body # This must be set `Rigid body`

platform:
    inertia: [ # inertia matrix of the spacecraft
        50000, 0, 0,
        0, 50000, 0,
        0, 0, 50000
        ]
```

`dynamicsmodel` should be set `Rigid body` to tell the software that you are using the rigid body dynamics model.

## Public interfaces

```@docs
RigidBodyModel
update_angularvelocity
```
