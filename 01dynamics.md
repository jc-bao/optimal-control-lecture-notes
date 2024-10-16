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

## Basic Concepts

**Response of Linear Time-Invariant (LTI) Systems**  

$$
\dot{x} = A x + B u \\
x(t) = e^{A(t-t_0)} x(t_0) + \int_{t_0}^{t} e^{A(t-s)} B u(s) ds
$$

**Gramian**  

$$
W(t_0, t) = \int_{t_0}^{t} e^{A(t-s)} B B^T e^{A^T (t-s)} ds
$$

$W(t_0, t) \xi = \lambda \xi$ if $\lambda$ is large, $\xi$ is easier to control.

**Controllability** 

$\dot{x} = A x + B u$

$C = \begin{bmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{bmatrix}$ (this is actually the impulse response matrix)

when $rank(C) = n$, the system is controllable.

In that case, we can place $A-BK$ to any desired eigenvalues.

**Reachability**  

$R_t = \set{\xi \mid ∃ u, \text{s.t.} x(t) = \xi}$

**Observability**  

$$
\dot{x} = A x + B u \\
y = C x
$$

$$
O = \begin{bmatrix} C & CA & CA^2 & \cdots & CA^{n-1} \end{bmatrix}
$$

when $rank(O) = n$, the system is observable.

**Kalman Filter**

$$
\dot{\hat{x}} = A \hat{x} + B u + K (y - C \hat{x})
$$

## Lyapunov Stability

Definition:

$$
V(x) \geq 0, \forall x \neq 0 \\
V(0) = 0 \\
\dot{V}(x) \leq 0, \forall x \neq 0
$$

## Hamilton Jacobi Bellman Equation

**The problem:**

$$
J(x(t), u(t), t_0, t_f) = \int_{t_0}^{t_f} l(x,u) dt + Q(x(t_f)) \\
V(x(t_0), t_0, t_f) = \min_u \left\{ \int_{t_0}^{t_f} l(x,u) dt + Q(x(t_f)) \right\}
$$

**The HJB equation:**

continous case:
$$
- \frac{\partial V}{\partial t} = \min_u \left\{ l(x,u) + \frac{\partial V}{\partial x} f(x,u) \right\}
$$

or:

$$
\begin{gather} 0 = \min_u \left[
      \ell(x,u) + \frac{\partial J^*}{\partial x} f(x,u) \right] \\
      \pi^*(x) = \argmin_u \left[ \ell(x,u) + \frac{\partial J^*}{\partial x} f(x,u) \right]
      \end{gather}
$$