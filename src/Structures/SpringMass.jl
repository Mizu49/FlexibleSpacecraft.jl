"""
    SpringMass

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module SpringMass

using LinearAlgebra

struct SpringMassModel
    # degrees of freedom (DOF) of the structure
    DOF
    # dimension of the control input vector
    dimcontrolinput
    # dimension of the disturbance input vector
    dimdistinput
    # transformation matrix. i.e. x = 𝚽𝛈, modal coordinates are mass-normalized
    𝚽
    # modal damping matrix
    𝚵
    # coupling matrix with the attitude dynamics (time derivative of the angular velocity vector)
    𝐃
    # disturbance input matrix
    𝐅
end

struct StateSpace
    𝐀 # system matrix
    𝐁 # control input matrix
    𝐄c # input matrix for the coupling part (subscript c represents coupling)
    𝐄d # input matrix for the disturbance input (subscript d represents disturbance)

    StateSpace(model::SpringMassModel) = begin
        𝐀 = [
            zeros(model.DOF) I
            -𝚽^2 -2*𝚵*𝚽
        ]
        𝐁 = [
            zeros(model.DOF, model.dimcontrolinput)
            model.controlinputmatrix
        ]
        𝐄c = [zeros(model.DOF, 3); 𝐃]
        𝐄d = [zeros(model.DOF, model.dimdistinput); model.𝐅]
    end
end

end
