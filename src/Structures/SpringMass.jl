"""
    SpringMass

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module SpringMass

using LinearAlgebra, StaticArrays

export physical2modal

"""
    PhysicalSystem

Representation of the structural system in physical coordinate

## Fields

* `dim::Integer`: dimension of the structural system
* `mass_matrix::Matrix`: mass matrix in physical coordinate
* `damping_matrix::Matrix`: damping matrix in physical coordinate
* `stiffness_matrix::Matrix`: stiffness matrix in physical coordinate
"""
struct PhysicalSystem
    dim::Integer

    mass_matrix::Matrix
    damping_matrix::Matrix
    stiffness_matrix::Matrix

    # Constructor
    PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix) = begin
        dim = size(mass_matrix, 1)
        new(dim, mass_matrix, damping_matrix, stiffness_matrix)
    end
end

"""
    ModalSystem

Representation of the structural system in modal coordinate

## Fields

* `dim::Integer`: dimension of the structural system
* `PHI::Matrix`: modal transformation matrix
* `OMEGA::Matrix`: modal angular velocity matrix
* `XI::Matrix`: modal damping matrix
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

struct SpringMassModel
    # degrees of freedom (DOF) of the structure
    DOF
    # dimension of the control input vector
    dimcontrolinput
    # dimension of the disturbance input vector
    dimdistinput
    # transformation matrix. i.e. x = 𝚽𝛈, modal coordinates are mass-normalized
    PHI
    # modal damping matrix
    XI
    # coupling matrix with the attitude dynamics (time derivative of the angular velocity vector)
    D
    # disturbance input matrix
    F
end

struct StateSpace
    sysA # system matrix
    sysB # control input matrix
    sysEc # input matrix for the coupling part (subscript c represents coupling)
    sysEd # input matrix for the disturbance input (subscript d represents disturbance)

    StateSpace(model::SpringMassModel) = begin
        sysA = [
            zeros(model.DOF) I
            -PHI^2 -2*XI*PHI
        ]
        sysB = [
            zeros(model.DOF, model.dimcontrolinput)
            model.controlinputmatrix
        ]
        sysEc = [zeros(model.DOF, 3); D]
        sysEd = [zeros(model.DOF, model.dimdistinput); model.F]
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

end
