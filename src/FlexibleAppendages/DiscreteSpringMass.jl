"""
    SpringMassParams

struct for accomodating the parameters for the spring mass structural model

# Fields

* `M::AbstractMatrix`: mass matrix
* `D::AbstractMatrix`: damping matrix
* `K::AbstractMatrix`: stiffness matrix
* `Ecoupling::AbstractMatrix`: coefficient matrix for the attitude coupling input
* `Econtrol::AbstractVecOrMat`: coefficient matrix for the control input
* `Edisturbance::AbstractVecOrMat`: coefficient matrix for the disturbance input

"""
struct SpringMassParams<:AppendagesBase.AbstractAppendageParameters
    # mass matrix
    M::AbstractMatrix
    # damping matrix
    D::AbstractMatrix
    # stiffness matrix
    K::AbstractMatrix
    # attitude coupling input coefficient matrix
    Ecoupling::AbstractMatrix
    # control input coefficient matrix
    Econtrol::AbstractVecOrMat
    # disturbance input coefficient matrix
    Edisturbance::AbstractVecOrMat
end

"""
    PhysicalSystem

Representation of dynamics part of the structural system in physical coordinate.

# Fields

* `dim::Integer`: dimension of the structural system
* `mass_matrix::Matrix`: mass matrix in physical coordinate
* `damping_matrix::Matrix`: damping matrix in physical coordinate
* `stiffness_matrix::Matrix`: stiffness matrix in physical coordinate

# Mathematical representation

This data containter corresponds to the left hand side of the equation of motion as follows:

```math
\\mathbf{M} \\ddot{\\mathbf{x}} + \\mathbf{D} \\dot{\\mathbf{x}} + \\mathbf{K} \\mathbf{x} = \\ldots
```

* ``\\mathbf{M}`` : mass matrix
* ``\\mathbf{D}`` : damping matrix
* ``\\mathbf{K}`` : stiffness matrix

# Constructor

```julia
PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)
```

# Example

```julia
# Suppose M, D, K are given

# Create representation of the system in physical coordinate
physicalsystem = PhysicalSystem(M, D, K)
```
"""
struct PhysicalSystem
    dim::Integer

    mass_matrix::Matrix
    damping_matrix::Matrix
    stiffness_matrix::Matrix

    # Constructor
    """
        PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)

    Constructor for data container `PhysicalSystem`
    """
    PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix) = begin
        dim = size(mass_matrix, 1)
        new(dim, mass_matrix, damping_matrix, stiffness_matrix)
    end
end

"""
    ModalSystem

Representation of dynamics part of the structural system in mass-normalized modal coordinate.

# Fields

* `dim::Integer`: dimension of the structural system
* `PHI::Matrix`: modal transformation matrix
* `OMEGA::Matrix`: modal angular velocity matrix
* `XI::Matrix`: modal damping matrix

# Mathematical representation

This data containter corresponds to the left hand side of the equation of motion as follows:

```math
\\ddot{\\mathbf{\\eta}} +  2 \\boldsymbol{\\Xi \\Omega} \\dot{\\mathbf{\\eta}} + \\boldsymbol{\\Omega}^2 \\mathbf{\\eta} = \\ldots
```

The modal transformation is given as:

```math
\\mathbf{x} = \\boldsymbol{\\Phi} \\boldsymbol{\\eta}
```

* ``\\boldsymbol{\\Phi}`` : transformation matrix from modal coordinate to physical coordinate
* ``\\boldsymbol{\\Omega}`` : modal angular velocity matrix
* ``\\boldsymbol{\\Xi}`` : modal damping ratio matrix

# Constructor

```julia
ModalSystem(PHI::Matrix, OMEGA::Matrix, XI::Matrix)
```

# Example

```julia
# You can convert the `physicalsystem::PhysicalSystem` into `::ModalSystem`
physicalsystem = PhysicalSystem(M, C, K)
# Convert representation of the system in modal coordinate
modalsystem = physical2modal(physicalsystem)
```
"""
struct ModalSystem
    dim::Integer

    PHI::Matrix
    OMEGA::Matrix
    XI::Matrix

    # Constructor
    ModalSystem(PHI::Matrix, OMEGA::Matrix, XI::Matrix) = begin
        dim = size(PHI, 1)
        new(dim, PHI, OMEGA, XI)
    end
end

"""
    SpringMassModel

Spring mass representation of the entire system modeling

# Mathematical representation

```math
\\ddot{\\mathbf{\\eta}} + 2 \\boldsymbol{\\Xi \\Omega} \\dot{\\mathbf{\\eta}} + \\boldsymbol{\\Omega}^2 \\mathbf{\\eta} = \\boldsymbol{\\Phi}^{\\mathrm{T}} \\mathbf{D} \\boldsymbol{\\omega} + \\boldsymbol{\\Phi}^{\\mathrm{T}} \\mathbf{F}_{\\mathrm{ctrl}} \\mathbf{f_\\mathrm{ctrl}} + \\boldsymbol{\\Phi}^{\\mathrm{T}} \\mathbf{F}_{\\mathrm{dist}} \\mathbf{f_\\mathrm{dist}}
```

# Fields

* `DOF::Integer`: dimension of the displacement vector of the system
* `dimcontrolinput::Integer`: dimension of the control input vector
* `dimdistinput::Integer`: dimension of the disturbance input vector
* `system::ModalSystem`: mass-normalized modal representation of the system
* `D::AbstractMatrix`: coupling matrix wiht the attitude motion (time derivative of the angular velocity vector)
* `Fctrl::AbstractVecOrMat`: coefficient matrix or vector of the control input vector
* `Fdist::AbstractVecOrMat`: coefficient matrix or vector of the disturbance input vector
"""
struct SpringMassModel
    # degrees of freedom of the system
    DOF::Integer
    # dimension of the control input vector
    dimcontrolinput::Integer
    # dimension of the disturbance input vector
    dimdistinput::Integer
    # Representation of the structural system
    system::ModalSystem

    # coupling matrix with the attitude dynamics (time derivative of the angular velocity vector)
    D::AbstractMatrix

    # control input matrix or vector
    Fctrl::AbstractVecOrMat
    # disturbance input matrix or vector
    Fdist::AbstractVecOrMat

    # Inner constructor for struct `SpringMassModel`
    SpringMassModel(system::ModalSystem, D::AbstractMatrix, Fctrl::AbstractVecOrMat, Fdist::AbstractVecOrMat) = begin
        # get dimension of the system
        DOF = system.dim
        dimcontrolinput = size(Fctrl, 2)
        dimdistinput = size(Fdist, 2)

        new(DOF, dimcontrolinput, dimdistinput, system, D, Fctrl, Fdist)
    end

    SpringMassModel(system::PhysicalSystem, D::AbstractMatrix, Fctrl::AbstractVecOrMat, Fdist::AbstractVecOrMat) = begin

        # convert physical system representation into modal system representation
        system = physical2modal(system)

        # get dimension of the system
        DOF = system.dim
        dimcontrolinput = size(Fctrl, 2)
        dimdistinput = size(Fdist, 2)

        new(DOF, dimcontrolinput, dimdistinput, system, D, Fctrl, Fdist)
    end
end

function physical2modal(physicalsystem::PhysicalSystem)::ModalSystem

    return physical2modal(physicalsystem.mass_matrix, physicalsystem.damping_matrix, physicalsystem.stiffness_matrix)
end

"""
    physical2modal(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)::ModalSystem

return tuple of the modal transformation matrix and modal damping matrix for the mass-normalized modal coordinates
"""
function physical2modal(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)::ModalSystem

    # dimension of the structure
    dim = size(mass_matrix, 1)

    # Eigen value decomposition
    # https://docs.julialang.org/en/v1/stdlib/LinearAlgebra/#LinearAlgebra.eigvecs
    PHI = eigvecs(mass_matrix, stiffness_matrix)

    # mass normalization
    mr = zeros(dim)
    for ind = 1:dim
        mr[ind] = PHI[:, ind]' * mass_matrix * PHI[:, ind]
        PHI[:, ind] = sqrt(1/mr[ind])*PHI[:, ind]

        # redo the calculation of modal mass, mr[idx] == 1.0
        mr[ind] = PHI[:, ind]' * mass_matrix * PHI[:, ind]
    end

    # natural angular frequency matrix
    # OMEGA is defined to be diagonal matrix of natural angular frequency, not its squared value
    OMEGA = sqrt(transpose(PHI) * stiffness_matrix * PHI)

    # calculate modal stiffness matrix Kr = omega^2
    kr = zeros(dim)
    for idx = 1:dim
        kr[idx] = OMEGA[idx, idx]^2
    end

    # modal damping ratio
    cr = zeros(dim)
    xi_r = zeros(dim)
    for idx = 1:dim
        cr[idx] = transpose(PHI[:, idx]) * damping_matrix * PHI[:, idx]
        xi_r[idx] = cr[idx]/(2 * sqrt(mr[idx] * kr[idx]))
    end
    XI = diagm(xi_r)

    # sort matrix with natural angular frequency
    sortidx = sortperm(diag(OMEGA))
    OMEGA = OMEGA[sortidx, sortidx]
    PHI = PHI[:, sortidx]
    XI = XI[sortidx, sortidx]

    return ModalSystem(PHI, OMEGA, XI)
end
