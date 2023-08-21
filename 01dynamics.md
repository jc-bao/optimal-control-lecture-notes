# Dynamics Introduction

📘 **Control Affine Systems**  
A control affine system is described by the differential equation:
$$
\dot{x} = f(x) + B(x)u
$$
where $\dot{x}$ is the rate of change of the state, $f(x)$ is the system's drift vector, $B(x)$ is the control matrix, and $u$ is the control input.

🤖 **Manipulator Dynamics**  
The dynamics of a robotic manipulator can be represented as:
$$
M(q) \dot{v} + C(q,v) = B(q)u + F
$$
Where:  
- $M(q)$: Mass matrix of the manipulator.
- $C(q,v)$: Dynamic bias term which includes potential energy-related effects.
- $B(q)$: Input Jacobian matrix.
- $F$: External forces acting on the manipulator.

Additionally, the velocity kinematics of the manipulator is given by:
$$
\dot{q} = G(q)v
$$

This representation is another form of the Euler-Lagrange equations, which can be derived from the Lagrangian:
$$
L = \frac{1}{2}v^T M(q) v - U(q)
$$
where $L$ is the Lagrangian, $v$ is the velocity, and $U(q)$ is the potential energy.

📐 **Linear Systems**  
A linear system can be described by:
$$
\dot{x} = A(t)x + B(t)u
$$
For a time-invariant system, the matrices $A(t)$ and $B(t)$ are constants, i.e., $A(t) = A$ and $B(t) = B$.

🎯 **Equilibria**  
Equilibrium points are states where the system remains unchanged over time. The stability of these points can be determined by analyzing the eigenvalues of the Jacobian matrix of the system. Specifically, an equilibrium is:
- **Stable** if the real parts of all eigenvalues of the Jacobian matrix are negative, i.e.,
$$
\textrm{Re} \big[ \text{eig} \left( \frac{\partial f}{\partial x} \right) \big] < 0
$$