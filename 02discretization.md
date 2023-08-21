# Discretization

## 📚 Discretization of Dynamics

### 📝 Explicit Form

- **Forward Euler Method**: This method is given by the equation:
    $$
    x_{k+1} = x_k + h f_\text{continuous} (x_k, u_k)
    $$

    - **Stability of the Discrete-Time System**: The stability can be analyzed by linearizing around the equilibrium point:
        $$
        \frac{\partial x_N}{\partial x_0} = \frac{\partial f_{\rm d}}{\partial x} \frac{\partial f_{\rm d}}{\partial x} ... {\frac{\partial f_{\rm d}}{\partial x}}{x_0} = A_{\rm d}^N
        $$
        The stability requirement is:
        $$
        | \textrm{eig} ( A_{\rm d})  | < 1
        $$

    - **Key Takeaways**:
        - 🚫 Always check the simulator energy.
        - ❌ Avoid using the Forward Euler integration as it can lead to instability.

- **4th-Order Runge-Kutta Method (RK4)**: This method is more accurate and is given by:
    $$
    x_{k+1} = f_{RK4} (x_k)
    $$
    Where:
    $$
    \begin{align*}
    K_1 & = f(x_k) \\
    K_2 & = f(x_k + \frac{1}{2} h K_1) \\
    K_3 & = f(x_k + \frac{1}{2} h K_2) \\
    K_4 & = f(x_k + h K_3) \\
    x_{k+1} & = x_{k} + \frac{h}{6} (K_1 + 2K_2 + 2K_3 + K_4)
    \end{align*}
    $$

    - **Key Takeaways**:
        - ✅ Provides better accuracy compared to the Forward Euler method.

### 📝 Implicit Form

- The implicit form is represented as:
    $$
    f_{\rm d} (x_{k+1}, x_{k}, u_k) = 0
    $$

- **Backward Euler Method**: This method is given by:
    $$
    x_{k+1} = x_{k} + h f(x_{k+1})
    $$
    This method results in energy damping.

    - **Key Takeaways**:
        - ✅ The implicit form is generally more stable.
        - ⏳ It is computationally more expensive.
        - 📌 In some "direct" trajectory optimization methods, they are *not* any more expensive to use!

## 📚 Discretization of Controls

- **Zero-Order Control**: This method holds the control constant over the interval:
    $$
    u(t) = u_k  \ \text{for} \ t_{k} \leq t \leq t_{k+1}
    $$

- **First-Order Control (Linear Interpolation)**: This method linearly interpolates the control over the interval:
    $$
    u(t) = u_{k} + \frac{u_{k+1}-u_k}{h} (t-t_n)
    $$