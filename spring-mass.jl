# 1DOF　バネマス系のシミュレーションプログラム
using Plots

function RK4(sysA, sysB, sysE, Ts, state, input1, input2)
    
    k1 = Ts * (sysA * state          + sysB * input1 + sysE * input2)
    k2 = Ts * (sysA * (state + k1/2) + sysB * input1 + sysE * input2)
    k3 = Ts * (sysA * (state + k2/2) + sysB * input1 + sysE * input2)
    k4 = Ts * (sysA * (state + k3/2) + sysB * input1 + sysE * input2)
    
    return state + 1/6*(k1 + 2*k2 + 2*k3 + k4)            
end

# パラメータ設定
m = 20
c = 1
k = 100

loopNUM = 200000
Ts = 1e-3

A = [
    0 1
    -k/m -c/m
]

B = [
    0
    1/m
]

E = [
    0
    1/m
]

stateArray = zeros(2, loopNUM)

stateArray[:,1] = [1;0]

for counter = 1:loopNUM-1
    stateArray[:,counter+1] = RK4(A, B, E, Ts, stateArray[:,counter], 0, 0)

    # println(stateArray[1, counter])
end

plot(0:Ts:Ts*(loopNUM-1), stateArray[1, :])

