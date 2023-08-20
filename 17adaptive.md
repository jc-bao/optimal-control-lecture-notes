# Lecture 17 Adaptive Control

## Ways to deal with uncertainties

- Feedback
  - Feedback is a fundamental concept in control systems. It refers to the process of adjusting the input based on the output to achieve desired behavior.
  - However, feedback alone might not be sufficient, especially when constraints are tight or when the system is complex.
- Improving the Model
    - Parameter Estimation:
       - Methods: System Identification (SystemID), Grey Box Modeling.
       - 🟢 Advantages: Sample efficient and generalizes well.
       - 🔴 Disadvantages: Assumes a good model structure.
    - Learning the Model:
       - Methods: Fit the model function or use residue black-box.
       - 🟢 Advantages: No assumption on model structure and generalizes well.
       - 🔴 Disadvantages: Sample inefficient and challenging to implement in real-world scenarios.

- Improving the Controller
    - Learning the Policy:
       - 🟢 Advantages: No assumptions on dynamics.
       - 🔴 Disadvantages: Suited for single tasks, doesn't generalize well, and is sample inefficient.
    - Improving a Trajectory:
       - Given a reference trajectory with a nominal model, feedback the real trajectory to the optimizer.
       - 🟢 Advantages: Makes few assumptions and is sample efficient.
       - 🔴 Disadvantages: Assumes a decent model and doesn't generalize well.

## 🔄 **Iterative Learning Control (ILC)**
- **Key Notes**:
  - ILC is a special case of the policy gradient on a policy class:
    $$
    u_k = \bar{u}_k - K_k(x_k - \bar{x}_k)
    $$
    The latter can be any controller.
  - ILC is also a special case of the SQP method, where the RHS vector comes from the system rollout.

- **Formulation of the Tracking Problem**:
  - The objective is to minimize the following cost function:
    $$
    \min_{x_{1:N}, u_{1:N}} J = \sum_{n=1}^{N-1} \left[ \frac{1}{2} (x_n - \bar{x}_n)^T Q (x_n - \bar{x}_n) + \frac{1}{2} (u_n - \bar{u}_n)^T R (u_n - \bar{u}_n) \right] + \frac{1}{2} (x_N - \bar{x}_N)^T Q_N (x_N - \bar{x}_N)
    $$
    Subject to:
    $$
    x_{n+1} = f_{nominal}(x_n)
    $$
    📝 **Note**: In a standard formulation, \(f_{nominal}\) should be \(f_{real}\), but we don't have access to \(f_{real}\). The idea is to replace the model-based rollout trajectory with the real rollout trajectory. However, in the real world, we cannot get the gradient, so we use the original model as an approximation.

- **Objective Gradient**:
  - The Lagrangian is defined as:
    $$
    L = J + \lambda^T c
    $$
    The gradient and Hessian of the KKT conditions are derived as:
    [Detailed mathematical derivations]

- **Important Observations**:
  - The constraint \(c(z) = 0\) is always satisfied with real-world data.
  - Given \(x_n, u_n\), we can still compute the gradient of \(J\).
  - The entire RHS of the KKT condition (gradient and state) can be obtained.
  - The controller can be polished using real-world data.
  - In practice, since \(x_n, u_n\) are already close to the reference trajectory by offline solving, we can compute \(C = \frac{\partial c}{\partial q} |_{\bar{x},\bar{u}}\).
  - We can then solve the KKT system for \(\Delta z\) and update \(\bar{u}\) using a QP method.
  - 📝 **Note**: During the rollout, we might use LQR to track the planned open-loop trajectory.

<div style="text-align: center;">
    <img src="figs/iterativeLearningControl.png" style="max-width: 600px; display: inline-block;">
</div>

- **Why Does ILC Work?**
  - ILC is an approximation of Newton's method (inexact/quasi-Newton method).
  - It allows for Newton-style optimization even if we don't have the exact gradient, as long as the approximation satisfies certain conditions.
  - The method might converge slower than the exact Newton method but is still effective.
