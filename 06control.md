# 📘 Deterministic Optimal Control

## 🕐 Continuous Time

The continuous time optimal control problem is defined as:

$$
\min_{x(t), u(t)} \int_{0}^{t_f} l(x,y) dt + l_F(x(t_f))
$$

subject to:

$$
\dot{x} = f(x,u)
$$

### 📝 Notes

- 📏 This is an infinite-dimensional problem. If the sample time becomes infinitely small, then \( u \) will have infinite dimensions.
- 🔄 Solutions are open-loop \( u(t) \):
  - 🎛 MPC uses the first few steps of open-loop \( u(t) \) (which does not apply in stochastic control).
  - 🔄 Other methods use offline solutions with feedback.
- ❓ Why optimize \( x \) and \( u \) at the same time instead of just \( x \)?
  - 🔄 When using forward simulation to get \( x_n \), the conditional number of \( A^N \) could be bad, making the optimization problem ill-posed.
  - 🤔 It's challenging to get a right \( u(t) \) initially. Providing a better initial \( x(t) \) aids the optimization process.

## 🕑 Discrete Time

The discrete-time optimal control problem is:

$$
\min_{x_{1:N}, u_{1:N-1}} \sum_{k=0}^{N-1} l(x_k,y_k) + l_F(x_N)
$$

subject to:

$$
x_{n+1} = f(x_n,u_n)
$$

$$
C(x_n) \le 0 \text{ (collision/obstacle constraints) }
$$

$$
u_{\text{min}} \le u_k \le u_{\text{max}} \text{ (control limit) }
$$

### 📝 Notes

- 📊 This is a finite-dimensional problem.
- 📍 \( x_n, u_n \) are called knot points.

# 📘 Pontryagin's Minimum Principle (PMP)

Also known as the maximum principle.

## 📝 Notes

- 📚 Provides first-order necessary conditions for deterministic optimal control problems.
- 🕑 In discrete time, it's just a special case of the KKT conditions.
- 📈 Can be seen as a generalization of the Euler-Lagrange equation for systems with control inputs.

## 🕑 For Discrete System

The Lagrangian is:

$$
L = \sum_{k=1}^{N-1} \left[ l(x_k,u_k) + \lambda_{k+1}^T (f(x_k,u_k)-x_{k+1}) \right] + l_F(x_N)
$$

The Hamiltonian is:

$$
H(x, u, \lambda) = l(x,u) + \lambda^T f(x,u)
$$

To solve it:

$$
\begin{align}
\frac{\partial L}{\partial \lambda_k} &= f(x_k,u_k) - x_{k+1} = 0 \\
\frac{\partial L}{\partial x_k} &= \frac{\partial l}{\partial x_k} + \lambda_{k+1}^T \frac{\partial f}{\partial x_k} - \lambda_k^T = 0 \\
\frac{\partial L}{\partial x_N} &= \frac{\partial l_F}{\partial x_N} - \lambda_N^T = 0
\end{align}
$$

For \( u \), considering torque limits:

$$
u_k = \arg\min_{\tilde{u}} H(x_k, \tilde{u}, \lambda_{k+1}) \ \text{such that} \ \tilde{u} \in \mathcal{U}
$$

The system dynamics and co-trajectory are:

$$
\begin{align}
x_{k+1} &= f(x_k, u_k) \\
\lambda_k &= \nabla_x l(x_k,u_k) + (\frac{\partial f}{\partial x})^T \lambda_{k+1} \\
u_k &= \arg\min_{\tilde{u}} H(x_k, \tilde{u}, \lambda_{k+1}) \ \text{such that} \ \tilde{u} \in \mathcal{U} \\
\lambda_{N} &= \frac{\partial l_F}{\partial X_{N}}
\end{align}
$$

❓ Why is \( u_k \) the argmin of \( H \)?

## 🕐 Continuous Time

The system dynamics in continuous time are:

$$
\begin{align}
\dot{x} &= f_{\text{continuous}}(x, u) \\
\dot{\lambda} &= \nabla_x l(x,u) + (\frac{\partial f_{\text{continuous}}}{\partial x})^T \lambda \\
u &= \arg\min_{\tilde{u}} H(x, \tilde{u}, \lambda) \ \text{such that} \ \tilde{u} \in \mathcal{U} \\
\lambda_{N} &= \frac{\partial l_F}{\partial X_{N}}
\end{align}
$$

## 🎯 Shooting Method

- 🚀 Start with an initial guess trajectory.
- 🔄 Simulate (or "rollout") to get \( x(t) \) using \( f(x,u) \).
- ⏪ Backward pass to get \( \lambda(t) \) and \( \Delta u(t) \) using the second equation.
- 🔄 Rollout with line search on \( \Delta u \) by optimizing the Hamiltonian.
- 🔁 Repeat from step 3 until convergence.
- ⚠️ Limitations: Not robust and can converge slowly.

## 📝 Notes

- 📜 Historically, many algorithms were based on forward/backward integration of the continuous ODEs for \( x(t) \) and \( \lambda(t) \) to perform gradient descent on \( u(t) \).
- 📏 These are called indirect and/or shooting methods (both are iterative methods).
- 🕐 In continuous time, \( \lambda(t) \) is called the "co-state" trajectory.
- 💻 These methods have largely fallen out of favor as computers and solvers have improved.


# ⭐ Algorithm Recap

In the realm of optimal control, understanding the nuances of different algorithms and their applications is crucial. Here's a comprehensive breakdown:

## 📏 Linear/Local Control Problems

Linear or local control problems can be categorized based on the presence or absence of constraints:

### 🚫 No Constraints

For problems without constraints, the Linear Quadratic Regulator (LQR) is typically used.

- **Time Variant (e.g., tracking)**: This is referred to as Time Variant LQR (TVLQR).
    - **Solution Approach**: Differential Dynamic Programming (DDP) or iterative LQR (iLQR) can be employed.
- **Time Invariant (e.g., stabilization)**: Known as Time Invariant LQR (TILQR).
    - **Solver Options**: Quadratic Programming (QP) or Dynamic Programming (DP) can be used. Both are equivalent in this context.

### ✅ With Constraints

Model Predictive Control (MPC) is the go-to for problems with constraints.

- **Linear Constraints**: Quadratic Programming (QP) is the solver of choice.
- **Conic Constraints**: Second Order Cone Programming (SOCP) is used.
- **Non-linear Constraints**: A non-linear optimizer is typically employed.

## 🌀 Non-linear Trajectory Optimization/Planning

The question arises: Can this be converted into TVLQR? Let's delve deeper:

### 🛠️ Methods

Two primary methods are used for non-linear trajectory optimization:

1. **Direct Collocation (DIRCOL)**: A direct method.
2. **Differential Dynamic Programming (DDP) or iterative LQR (iLQR)**: An indirect method.

Both methods are designed to tackle similar problems, but they have distinct characteristics:

| Terms                    | DIRCOL                                                                                          | DDP/iLQR                                                                                                   |
|--------------------------|-------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|
| Dynamic Feasibility      | Only satisfied when converged.                                                                  | Always maintained with rollout, allowing for early stopping and deployment.                                |
| Warm Start               | Can use an initial guess.                                                                       | Cannot use a warm start.                                                                                   |
| Dynamic Constraints      | Can handle any constraints, depending on the backend solver.                                    | Handles constraints with modifications: For `u`: Use squashing or constrained QP. For `x`: Add extra cost or use augmented Lagrangian. |
| Output                   | Produces only an open-loop trajectory, requiring an additional controller.                       | The converged feedback term provides a built-in controller.                                                |
| Speed                    | Slower as it solves the entire problem at once.                                                 | Faster due to simplified subproblems.                                                                      |
| Implementation Difficulty| Harder to implement.                                                                            | Easier to implement.                                                                                       |
| Numeric Stability        | Robust.                                                                                        | Long horizons can lead to ill-conditioning.                                                                |



### 🤔 How to Choose Between DIRCOL and DDP/iLQR?

- **Online/Real-time Control**: DDP is preferable since speed is paramount and constraint tolerance can be slightly relaxed.
- **Offline Trajectory Generation & Long-horizon Problems**: DIRCOL is more suitable.
- **Multi-shooting Approach**: Use DDP for subtrajectory rollouts (with reduced horizons) and combine them with constraints solved by DIRCOL (simplifying the problem).

In conclusion, the choice of algorithm largely depends on the specific requirements and constraints of the problem at hand. By understanding the strengths and weaknesses of each method, one can make informed decisions in the realm of optimal control.