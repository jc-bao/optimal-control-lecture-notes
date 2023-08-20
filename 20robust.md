# 📘 Robust Control

## 🌐 Context

- Traditional stochastic methods primarily assume random noise.
    - 🚫 They often fail to account for parametric uncertainty or unmodeled dynamics.
- Iterative Learning Control (ILC) falls under the general category of adaptive control. It adapts to unknown models to achieve optimal control.
- 🛡️ Robust control offers a more conservative approach. It aims to fit all possible models, often trading off robustness against optimality.

## 🚧 Problem with LQG

- Linear Quadratic Gaussian (LQG) control can be highly fragile in the presence of model uncertainty.

## 📝 Problem Formulation

Consider the following system dynamics:

$$
x_{n+1} = f(x_n, u_n, w_n)
$$

Here, the disturbance \( w_n \) could represent constant parameters, time-varying non-smooth forces, or even model errors. We make an assumption that:

$$
||w|| < \epsilon
$$

The optimization problem can be formulated as:

$$
\begin{align*}
\min_{x_{1:N}, u_{1:N}} \max_{w_{1:N}} J &= \sum_{n=1}^{N-1} l_n(x_n, u_n, w_n) + l_N(x_N) \\
\text{s.t.} \ \ x_{n+1} &= f(x_n, u_n, w_n) \\
x_n &\in \mathcal{X} \\
u_n &\in \mathcal{U} \\
w_n &\in \mathcal{W}
\end{align*}
$$

This is a min-max optimization problem, which is typically challenging to solve.

## 🌀 Non-linear Systems and \( H_\infty \) Control

For non-linear systems, one can use \( H_\infty \) control, which is a generalized form of LQG. Local approximation solutions are also viable.

## 🛠️ Minimax DDP (Differential Dynamic Programming)

Minimax DDP is another variation of LQR tailored for robust control. The approach uses local/linear/quadratic Taylor expansion to iteratively find a locally-optimal trajectory and feedback policy.

Given:

$$
f(x+\Delta x, u+\Delta u, w+\Delta w) \approx f(x, u, w) + A\Delta x + B\Delta u + D\Delta w
$$

The action-value function can be expressed as:

$$
\begin{align*}
S(x+\Delta x, u+\Delta u, w+\Delta w) &\approx S(x, u, w) + \begin{bmatrix} g_x \\ g_u \\ g_w \end{bmatrix}^T \begin{bmatrix} \Delta x \\ \Delta u \\ \Delta w \end{bmatrix} \\
&+ \frac{1}{2} \begin{bmatrix} \Delta x \\ \Delta u \\ \Delta w \end{bmatrix}^T \begin{bmatrix} G_{xx} & G_{xu} & G_{xw} \\ G_{ux} & G_{uu} & G_{uw} \\ G_{wx} & G_{wu} & G_{ww} \end{bmatrix} \begin{bmatrix} \Delta x \\ \Delta u \\ \Delta w \end{bmatrix}
\end{align*}
$$

Using the Bellman equation (cost-to-go function):

$$
\begin{align*}
V_{n-1}(x+\Delta x) &= \min_{\Delta u} \max_{\Delta w} [S(x, u, w) + g_x^T\Delta x + g_u^T\Delta u + g_w^T\Delta w] \\
&+ \frac{1}{2} \Delta x^T G_{xx} \Delta x + \frac{1}{2} \Delta u^T G_{uu} \Delta u + \frac{1}{2} \Delta w^T G_{ww} \Delta w \\
&+ \Delta x^T G_{xu} \Delta u + \Delta u^T G_{uw} \Delta w + \Delta x^T G_{xw} \Delta w
\end{align*}
$$

From the gradient:

$$
\begin{align*}
\frac{\partial V}{\partial u} &= g_u + G_{uu} \Delta u + G_{ux} \Delta x + G_{uw} \Delta w = 0 \\
\frac{\partial V}{\partial w} &= g_w + G_{ww} \Delta w + G_{wx} \Delta x + G_{wu} \Delta u = 0
\end{align*}
$$

The solution can be derived as:

$$
\begin{align*} \Rightarrow \Delta u &= -d -K\Delta x\\ d&=(G_{uu}-\underbrace{G_{uw}G^{-1}_{ww}G_{wu}}_{\text{robust term}})^{-1}(g_u-\underbrace{G_{uw}G^{-1}_{ww}g_w}_{\text{robust term}})\\ K&=(G_{uu}-\underbrace{G_{uw}G^{-1}_{ww}G_{wu}}_{\text{robust term}})^{-1}(G_{ux}-\underbrace{G_{uw}G^{-1}_{ww}G_{wu}}_{\text{robust term}})) \end{align*}
$$

$G_{ww}$ is the Hessian of the cost of adversary. A large G implies expensive adversary action, so the adversiary is elimiated, which become standard DDP. 

For the adversary:

$$
\begin{align*}
\Delta w &= -e - L\Delta x \\
e &= (G_{ww} - G_{wu} G^{-1}_{uu} G_{uw})^{-1} (g_w - G_{wu} G^{-1}_{uu} g_u) \\
L &= (G_{ww} - G_{wu} G^{-1}_{uu} G_{uw})^{-1} (G_{wx} - G_{wu} G^{-1}_{uu} G_{ux})
\end{align*}
$$

The new value function is:

$$
\begin{align*}
V_{n-1}(x+\Delta x) &= V_{n-1}(x) + p_{n-1}^T \Delta x + \frac{1}{2} \Delta x^T P_{n-1} \Delta x \\
p_{n-1} &= g_x - G_{xu} G^{-1}_{uu} g_u - G_{xw} G_{ww}^{-1} g_w \\
P_{n-1} &= G_{xx} - G_{xu} G^{-1}_{uu} G_{ux} - G_{xw} G_{ww}^{-1} G_{wx}
\end{align*}
$$

A significant difference with standard DDP is the requirement for \( \text{dim}(x) + \text{dim}(u) \) number of positive eigenvalues and \( \text{dim}(w) \) negative eigenvalues.

For the quadratic programming, the equation should be:

$$
\begin{align*}
J = \sum_{n=1}^{N-1} \left[ \frac{1}{2} x_n^T Q x_n + \frac{1}{2}u_n^T R u_n + \frac{1}{2} w_n^T W w_n \right] + \frac{1}{2} x_N^T Q_N x_N \\
&Q, Q_N, R > 0 \ \ \ \ \ W < 0
\end{align*}
$$

Here, a larger \( ||w|| \) means more robustness. As \( ||w|| \) approaches infinity, the problem converges to the standard DDP.

## 🎓 Conclusion

Robust control, particularly through the use of Minimax DDP, provides a powerful framework for handling uncertainties in control systems. By considering both the control and disturbance variables, it offers a more conservative and reliable approach compared to traditional stochastic methods.

The inclusion of robust terms in the equations ensures that the system can handle uncertainties in both the model parameters and external disturbances. This makes it suitable for real-world applications where perfect knowledge of the system is often unattainable.

The mathematical formulation and iterative methods used in Minimax DDP allow for the fine-tuning of the trade-off between robustness and optimality, providing a flexible tool for control system designers.