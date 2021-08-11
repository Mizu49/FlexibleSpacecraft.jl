# Frames

We need several coordinate frames to express spacecraft attitude dynamics.

## ECI (Earth-Centered Inertial) Frame

Inertial frame of our dynamics. Always fixed. Attitude dynamics are usually described based on this frame.

* Origin: center of the Earth
* X-axis: direction of crossing between the equator and latitude 0 degrees at the initial state
* Y-axis: direction of crossing between the equator and longitude +90 degrees at the initial state
* Z-axis: direction corresponding to the Earth's rotation axis

## ECEF (Earth-Centered Earth-fixed) Frame

Rotating coordinate frame according to the Earth's rotation. This frame is used mainly to express the ground equipment on the Earth.

* Origin: center of the Earth
* X-axis: longitude 0 degrees on Greenwich meridian
* Y-axis: longitude 90 degrees east
* Z-axis: direction corresponding to the Earth's north pole

## LVLH (Local Vertical Local Horizontal) frame

Referential frame of spacecraft on orbit. This frame describes the motion of spacecraft on orbit. And spacecraft attitude (Spacecraft-fixed frame) is expressed with respect to this frame.

* Origin: center of the spacecraft
* X-axis: direction of travel on orbit (roll axis)
* Y-axis: orthogonal direction to orbit plane (pitch axis)
* Z-axis: direction to the Earth (yaw axis)
## Body frame (Spacecraft-fixed frame)

Frame that is fixed to the spacecraft body. Describing the attitude of spacecraft.

* Origin: center of the spacecraft or referential point on spacecraft
* X-axis: X-axis of the spacecraft body
* Y-axis: Y-axis of the spacecraft body
* Z-axis: Z-axis of the spacecraft body
