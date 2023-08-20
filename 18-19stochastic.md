# Stochastic Optimal Control 📊

In many real-world systems, we cannot measure the system state perfectly. Instead, we often encounter situations where we can only obtain quantities related to the state through a measurement model:

📝 **Measurement Model**:
$$
y = g(x)
$$

The primary goal in stochastic optimal control is to determine a policy that minimizes the expected value of a certain objective function:

🎯 **Objective Function**:
$$
\min_{\pi_\theta} E[J(x,u)]
$$

While this could theoretically be solved using Quadratic Programming (QP), it's generally challenging due to the inherent complexities of stochastic systems.

## LQG: Linear Quadratic Gaussian Control 🌀

LQG is a special case of the stochastic control problem for which we can derive a closed-form solution. It combines the principles of Linear Quadratic Regulation (LQR) with Gaussian noise.

📐 **System Dynamics**:
$$
\begin{align*}
x_{n+1} &= Ax_n + Bu_n + w_n \quad &w_n \sim N(0,W) \\
y_n &= Cx_n + v_n \quad &v_n \sim N(0,V)
\end{align*}
$$

🎯 **Cost Function**:
$$
\begin{align*}
J = E\left[x_N^TQ_Nx_N + \sum_{n=1}^{N-1} x_n^TQx_n + u_n^TRu_n\right]
\end{align*}
$$

To solve the LQG problem, we can employ Dynamic Programming (DP) recursion:

🔄 **DP Recursion**:
$$
\begin{align*} V_N(x)=&E[x_N^TQ_Nx_N]=E[x_N^TP_Nx_N] \\
V_{N-1}(x)=&\min_uE[x_{N-1}^TQx_{N-1}+u^T_{N-1}Ru_{N-1}\\&+(Ax_{N-1}+Bu_{N-1}+w_{N-1})^TP_N(Ax_{N-1}+Bu_{N-1}+w_{N-1})]\\ =&\min_uE[\underbrace{x_{N-1}^TQx_{N-1}+u^T_{N-1}Ru_{N-1}+(Ax_{N-1}+Bu_{N-1})^TP_N(Ax_{N-1}+Bu_{N-1})}_{\text{standard LQR}}]\\ &+E[\underbrace{(Ax_{N-1}+Bu_{N-1})^TP_Nw_{N-1}+w_{N-1}^TP_N(Ax_{N-1}+Bu_{N-1})}_{\text{uncorrelated}}+\underbrace{w_{N-1}^TP_Nw_{N-1}}_{\text{constant}}]\\ =&\min_uE[\underbrace{x_{N-1}^TQx_{N-1}+u^T_{N-1}Ru_{N-1}+(Ax_{N-1}+Bu_{N-1})^TP_N(Ax_{N-1}+Bu_{N-1})}_{\text{standard LQR}}]\\ &+\underbrace{w_{N-1}^TP_Nw_{N-1}}_{\text{constant}}]] 
\end{align*}
$$

The solution to the LQG problem involves performing standard LQR but with state estimation:

🔍 **State Estimation**:
$$
\begin{align*}
u^T R + (A E[x_{N-1}] + B u)^T Q B &= 0 \\
\implies u_{N-1} &= -(R + B^T Q_N B)^{-1} B^T Q_N A E[x_{N-1}] \\
&= -K_{N-1}E[x_{N-1}]
\end{align*}
$$

In summary, the LQG control strategy provides a systematic approach to handle systems with Gaussian noise, combining the robustness of LQR with the adaptability required for stochastic environments.

## 📚 Optimal Estimation

### 🎯 Objective of Optimization

- **MAP (Maximum a Posteriori Estimation)**:
  
    $$ \hat{x} = \arg\max_x p(x|y) $$

- **MMSE (Minimum Mean-Squared Error)**:
  
    $$
    \begin{align*}
    \hat{x} &= \arg\min_{\hat{x}} E[(x-\hat{x})^T(x-\hat{x})] \\
    &= \arg\min_{\hat{x}} E[tr((x-\hat{x})^T(x-\hat{x}))] \\
    &= \arg\min_{\hat{x}} E[(x-\hat{x})(x-\hat{x})^T] \\
    &= \arg\min_{\hat{x}} tr(\Sigma)
    \end{align*}
    $$

### 🔄 Kalman Filter

The Kalman Filter is a recursive linear MMSE estimator:

$$ \hat{x}_{n|k} = \mathbb{E}[x_n | y_{1:k}] $$

- **Prediction Step**:
  
    $$
    \hat{x}_{n+1|n} = A\hat{x}_{n|n} + Bu_n
    $$
    
    $$
    \Sigma_{n+1|n} = A\Sigma_{n|n}A^T + W
    $$

- **Measurement Update**:

    The error signal is fed into the estimator to update (the innovation):

    $$
    z_{n+1} = y_{n+1} - C\hat{x}_{n+1|n}
    $$

    $$
    S_{n+1} = C\sigma_{n+1|n}C^T + V
    $$

- **State Update with Kalman Gain** (can run at different frequencies):

    $$ \hat{x}_{n+1|n+1} = \hat{x}_{n+1|n} + L_{n+1}z_{n+1} $$

- **Covariant Update with Joseph Form**:

    $$
    \Sigma_{n+1|n+1} = (I-L_{n+1}C)\Sigma_{n+1|n} (I-L_{n+1}C)^T + L_{k+1}VL_{k+1}
    $$

- **Kalman Gain**:

    $$
    L_{n+1} = \Sigma_{n+1|n}C^TS^{-1}_{n+1}
    $$

- ⭐ **Kalman Filter Algorithm Summary**:

    **Initialize**:
    
    $$
    \hat{x}_{0|0}, \Sigma_{0|0}, W, v
    $$
    
    **Predict**:
    
    $$
    \hat{x}_{n+1|n} = A\hat{x}_{n|n} + Bu_n
    $$
    $$
    \Sigma_{n+1|n} = A\Sigma_{n|n}A^T + W
    $$
    
    **Calculate Innovation + Covariance**:
    
    $$
    z_{n+1} = y_{n+1} - C\hat{x}_{n+1|n}
    $$
    $$
    S_{n+1} = C\sigma_{n+1|n}C^T + V
    $$
    
    **Calculate Kalman Gain**:
    
    $$
    L_{n+1} = \Sigma_{n+1|n}C^TS^{-1}_{n+1}
    $$
    
    **Update**:
    
    $$
    \hat{x}_{n+1|n+1} = \hat{x}_{n+1|n} + L_{n+1}z_{n+1}
    $$
    $$
    \Sigma_{n+1|n+1} = (I-L_{n+1}C)\Sigma_{n+1|n} (I-L_{n+1}C)^T + L_{k+1}VL_{k+1}
    $$

### 📝 Prediction Notes

- If measurements occur at different rates, careful tracking is needed. For instance, if one measurement is at 10 Hertz and another at 50 Hertz, predictions should be run at 50 Hertz for the latter, and possibly at 10 Hertz for the former.
- If a system is unobservable, the covariance will become unbounded over time since measurement updates reduce state covariance, while prediction steps increase it.
- **Particle Filters and Multimodal Distributions**: For truly multimodal statistics, particle filters are recommended. Gaussian mixture model filters or blob filters can be used if the distribution is known to be bimodal.

### 🌀 For Non-linear Cases: Extended Kalman Filter

## 🔄 Duality + Trajectory Optimization

MMSE estimation equivalent control formulation:

$$
\begin{align*}
&\min_{x_{1:N}, w_{1:N}} \frac{1}{2}(y_n - g(x_n))^TV^{-1}(y_n - g(x_n)) + \frac{1}{2}w_n^TW^{-1}w \\
&\text{s.t} \ \ \ \ \ \ \ x_{n+1} = f(x_n) + w_n
\end{align*}
$$

Here, noise is treated as control and \( g(x) \) as state. The goal is to keep the state close to the real state and also keep the control small.