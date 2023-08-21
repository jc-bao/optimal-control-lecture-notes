# Direct Trajectory Optimization 🚀

Direct Trajectory Optimization offers an alternative approach to solving non-linear trajectory optimization problems.

## What is Trajectory Optimization? 🤔

The output control form of trajectory optimization can be either:
- An open-loop control policy,
- A feedback policy, or
- It might depend on the solver being used.

## Different Strategies 🛠

One common strategy is to discretize and convert the continuous-time optimal control problem into nonlinear programming (NLP). This is done using the following standard non-linear programming formulation:

$$
\min_x f(x) \\
\text{s.t. } C(x) = 0 \; d(x) \le 0
$$

Assumptions:
- The function is \(C^2\) smooth.

## Sequential Quadratic Programming (SQP) 📈

SQP is a standard method for trajectory optimization. 

### Off-the-shelf NLP Solvers 🖥

- IPOPT (open source)
- SNOPT (commercial)
- KNITRO (commercial)

### Strategy 🧠

SQP works by converting the NLP problem into a QP (Quadratic Programming) problem. This is achieved using the 2nd order Taylor expansion of the Lagrangian and linearized \(C(x)\) and \(d(x)\) to approximate NLP as QP.

$$
\begin{align*}
 \min_{\Delta x} & \frac{1}{2} \Delta x^T H \Delta x + g^T \Delta x \\
 \text{s.t. } & c(x)+ C \Delta x = 0 \\
 & d(x) + D \Delta x \leq 0 \\
\end{align*}
$$

Where:
- \(H = \frac{\partial L}{\partial x}\)
- \(g = \frac{\partial L}{\partial x}\)
- \(C = \frac{\partial c}{\partial x}\)
- \(D = \frac{\partial d}{\partial xb}\)
- \(L(x, \lambda, \mu) = f(x) + \lambda^T c(x) + \mu^T d(x)\)

### Solving the QP 🧮

To solve the QP, compute the primal-dual search direction:

$$
\Delta z = \begin{bmatrix}
\Delta x \\
\Delta \lambda \\
\Delta \mu \end{bmatrix}
$$

Then, perform a line search with a merit function. 

For classical QP with only equality constraints, Newton’s method can be applied directly by solving the KKT condition:

$$
\begin{align*}
\begin{bmatrix} H & C^T \\ C & 0 \end{bmatrix} \begin{bmatrix} \Delta x \\ \Delta \lambda \end{bmatrix} = \begin{bmatrix} -\frac{\partial L}{\partial x} \\ -c(x) \end{bmatrix}
\end{align*}
$$

### SQP Insights 🧐

- SQP can be viewed as a generalization of Newton’s method to sequential constrained problems.
- Any QP solver should work, but implementation details, including warm starts, matter.
- Good performance leverages sparsity.
- For convex inequality constraints, it can be generalized to SCP (sequential convex programming).

## Direct Collocation 📍

Direct Collocation is an efficient way to impose dynamic constraints.

### Purpose of Direct Collocation 🤷

The main goal is to reduce the number of optimization variables. Instead of integrating, constraints are imposed with higher-order splines.

### Direct Collocation Insights 🧐

- In the indirect method, a rollout is needed to make dynamics work in a sequential decision process.
- In the direct method, dynamics can be enforced into a single equality since all states and controls are considered in one-time optimization.
- Direct collocation uses polynomial splines to represent trajectories and enforces dynamics and spline derivatives.
- The method achieves 3rd order integration accuracy, similar to RK3.
- It requires fewer dynamics calls and can use larger timesteps, saving up to 80% of the time.
- However, it is sensitive to warm starts.

## Conclusion 🎓

Direct Trajectory Optimization offers a robust and efficient way to solve non-linear trajectory optimization problems. By understanding the underlying strategies and methods, one can effectively apply these techniques in real-world applications.