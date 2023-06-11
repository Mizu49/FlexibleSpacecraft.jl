"""
    StateSpace

State space representation of the structural system. This representation is mainly used for the time evolution of the structural system

# Mathematical representation

State space equation of the structural system with the control input, coupling input (angular velocity of the attitude dynamics), and the disturbance input is given as:

```math
\\frac{d}{dt} \\mathbf{z} = \\mathbf{A} \\mathbf{z} + \\mathbf{B} \\mathbf{u} + \\mathbf{E}_{\\mathrm{cplg}} \\boldsymbol{\\omega} + \\mathbf{E}_{\\mathrm{dist}} \\mathbf{f}_{\\mathrm{dist}}
```

Each matrix is defined as follows:

```math
\\mathbf{A} \\equiv \\begin{bmatrix}
    \\mathbf{0} & \\mathbf{I} \\\\
    - \\boldsymbol{\\Omega}^2 & -2 \\boldsymbol{\\Xi \\Omega}
\\end{bmatrix}
\\\\
\\mathbf{B} \\equiv \\begin{bmatrix}
    \\mathbf{0} \\\\
    \\mathbf{F}_{\\mathrm{ctrl}}
\\end{bmatrix}
\\\\
\\mathbf{E}_{\\mathrm{cplg}} \\equiv \\begin{bmatrix}
    \\mathbf{0} \\\\
    \\mathbf{D}
\\end{bmatrix}
\\\\
\\mathbf{E}_{\\mathrm{dist}} \\equiv \\begin{bmatrix}
    \\mathbf{0} \\\\
    \\mathbf{F}_{\\mathrm{dist}}
\\end{bmatrix}
```

# Fields

* `dimstate::Int`: dimension of the state vector
* `dimctrlinput::Int`: dimension of the control input vector
* `dimdistinput::Int`: dimension of the disturbance input vector
* `sysA::SMatrix`: system matrix
* `sysB::StaticArray`: coefficient matrix or vector for control input
* `sysEcplg::SMatrix`: input matrix for the coupling part (subscript represents coupling)
* `sysEdist::StaticArray`: coefficient matrix or vector for the disturbance input (subscript represents disturbance)

# Constructor

```julia
StateSpace(model::SpringMassModel)
```

"""
struct StateSpace<:AppendagesBase.AbstractAppendageModel

    # dimension of the state and input vectors
    DOF::Int
    dimstate::Int
    dimctrlinput::Int
    dimdistinput::Int

    sysA::SMatrix # system matrix
    sysB::StaticArray # control input matrix
    sysEcplg::SMatrix # input matrix for the coupling part (subscript represents coupling)
    sysEdist::StaticArray # input matrix for the disturbance input (subscript represents disturbance)

    modalstate2physicalstate::SMatrix

    StateSpace(model::SpringMassModel) = begin

        dimstate = 2 * model.DOF
        dimctrlinput = model.dimcontrolinput
        dimdistinput = model.dimdistinput

        sysA = SMatrix{dimstate, dimstate}([
            zeros(model.DOF, model.DOF) I
            -model.system.OMEGA^2 -2 * model.system.XI * model.system.OMEGA
        ])

        # Need to switch the definition (`SVector` or `SMatrix`) based on the dimension
        if dimctrlinput == 1
            sysB = SVector{dimstate}([
                zeros(model.DOF)
                model.Fctrl
            ])
        else
            sysB = SMatrix{dimstate, dimctrlinput}([
                zeros(model.DOF, model.dimcontrolinput)
                model.Fctrl
            ])
        end

        sysEcplg = SMatrix{dimstate, 3}([zeros(model.DOF, 3); model.D])

        # Need to switch the definition (`SVector` or `SMatrix`) based on the dimension
        if dimdistinput == 1
            sysEdist = SVector{dimstate}([zeros(model.DOF); model.Fdist])
        else
            sysEdist = SMatrix{dimstate, dimdistinput}([zeros(model.DOF, model.dimdistinput); model.Fdist])
        end

        # transformation matrix for state vector in modal coordinate to physical coordinate
        modalstate2physicalstate = SMatrix{dimstate, dimstate, Real}([
            model.system.PHI zeros(model.DOF, model.DOF)
            zeros(model.DOF, model.DOF) model.system.PHI
        ])

        new(model.DOF, dimstate, dimctrlinput, dimdistinput, sysA, sysB, sysEcplg, sysEdist, modalstate2physicalstate)
    end
end

"""
    updatestate(model::StateSpace, Tsampling::Real, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

Calculates time evolution of the structural system with Runge-Kutta method
"""
function updatestate(model::StateSpace, Tsampling::Real, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

    # Runge-Kutta method
    k1 = _calc_differential(model, currenttime              , currentstate                   , angularvelocity, controlinput, distinput)
    k2 = _calc_differential(model, currenttime + Tsampling/2, currentstate + Tsampling/2 * k1, angularvelocity, controlinput, distinput)
    k3 = _calc_differential(model, currenttime + Tsampling/2, currentstate + Tsampling/2 * k2, angularvelocity, controlinput, distinput)
    k4 = _calc_differential(model, currenttime + Tsampling  , currentstate + Tsampling   * k3, angularvelocity, controlinput, distinput)
    nextstate = currentstate + Tsampling/6 *(k1 + 2*k2 + 2*k3 + k4)

    return nextstate
end

function _calc_differential(model::StateSpace, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

    diff = model.sysA*currentstate + model.sysB*controlinput + model.sysEcplg*angularvelocity + model.sysEdist*distinput

    return diff
end
