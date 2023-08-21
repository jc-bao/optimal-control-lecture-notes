# 🚀 3D rotation in modelling and optimization

In this section, we will delve into the intricacies of 3D rotation, specifically focusing on control within the SO(3) space.

## 🌐 Basic SO(3) Concepts

### 1️⃣ Rotation Matrix

- **Attitude Definition**: The attitude of an object in 3D space can be defined using the equation:
  
  $$
  ^N V = Q ^B V
  $$

  Here:
  - $^N V$ represents the vector in the world frame.
  - $^B V$ represents the vector in the body frame.
  - $Q$ is the mapping matrix that transforms the body frame to the world frame. It is an orthogonal matrix.

  ![Attitude Visualization](CMU16-745%20Optimal%20Control%20b4017ef3591745c2b0f29777c17a99ff/Untitled%206.png)

  The transformation can be further expanded as:

  $$
  \begin{align*}
  \begin{bmatrix}
  ^N x_1 \\
  ^N x_2 \\
  ^N x_3
  \end{bmatrix}
  =
  Q
  \begin{bmatrix}
  ^B x_1 \\
  ^B x_2 \\
  ^B x_3
  \end{bmatrix}
  \end{align*}
  $$

  - **Interpretation**:
    - Rows of the matrix represent the projection in the body frame.
    - Columns represent the combination in the world frame.

- **Integration of Attitude**:

  The rate of change of the transformation matrix with respect to time can be represented as:

  $$
  \dot{Q} = Q \ \hat{^B\omega}
  $$

  or

  $$
  \dot{Q} = \hat{^N\omega} \ Q
  $$

  Deriving this equation involves both geometric and derivative perspectives. The geometric perspective can be understood as:

  $$
  ^N x = Q(t) ^B x
  $$

  This leads to:

  $$
  \omega \times x = \hat \omega x
  $$

  And from the derivative perspective:

  $$
  ^N x = Q ^B x \Rightarrow ^N \dot x = \dot Q ^B x 
  $$

  Comparing the two gives:

  $$
  \Rightarrow  \dot Q ^B x = Q \hat \omega ^B x \Rightarrow \dot Q = Q \hat w
  $$

### 2️⃣ Axis-Angle Vector

The axis-angle representation is given by:

$$
\phi = a \theta
$$

### 3️⃣ Quaternion

A more compact way to represent 3D rotations is using quaternions. The quaternion representation is:

$$
q = e^{\frac{\theta}{2}(a_xi+a_yj+a_zk)} = \cos(\frac{\theta}{2}) + (a_xi+a_yj+a_zk) \sin(\frac{\theta}{2})
$$

- **Quaternion Multiplication**:

  The multiplication of two quaternions can be represented as:

  $$
  q_1 * q_2 = L(q_1) \begin{bmatrix} s_2 \\ v_2 \end{bmatrix} = R(q_2) \begin{bmatrix} s_1 \\ v_1 \end{bmatrix}
  $$

- **Quaternion Rotation**:

  The rotation using quaternions can be represented as:

  $$
  q*p*q^{-1} = R^T(q) L(q) Hx
  $$

- **Quaternion Kinematics**:

  The kinematics of quaternions can be represented as:

  $$
  \dot{q} = \frac{1}{2} L(q) H \omega
  $$

  This equation provides a way to rotate the angular velocity (in the body frame) to the world frame.

- **Useful Mathematical Properties**:

  Several properties and equations are essential when working with rotations:

  $$
  S(\boldsymbol{x}) = -S(\boldsymbol{x})^{\top}
  $$

  $$
  S(\boldsymbol{x}) \boldsymbol{y} = \boldsymbol{x} \times \boldsymbol{y}
  $$

  $$
  S(\boldsymbol{R} \boldsymbol{x}) = \boldsymbol{R} S(\boldsymbol{x}) \boldsymbol{R}^{\top}
  $$

  $$
  S(\boldsymbol{x}) \boldsymbol{x} = \mathbf{0}
  $$

  Additionally, the relationship between the rotation matrix and the rotation axis and angle can be represented as:

  $$
  \boldsymbol{R} = \cos \rho \cdot \boldsymbol{I} + (1-\cos \rho) \boldsymbol{n} \boldsymbol{n}^{\top} + \sin \rho \cdot S(\boldsymbol{n})
  $$

  $$
  \operatorname{tr}(\boldsymbol{R}) = 2 \cos \rho + 1
  $$

  $$
  \frac{d}{d t} \operatorname{tr}(\boldsymbol{R}) = -2 \sin \rho \cdot \boldsymbol{n}^{\top} \boldsymbol{\omega}
  $$

In conclusion, understanding the basics of SO(3) is crucial for attitude control in 3D space. The concepts of rotation matrices, axis-angle vectors, and quaternions provide the necessary mathematical tools to represent and manipulate 3D rotations effectively.

## 📚 Optimizing Quaternion

When working with quaternions in the context of optimization, there are several nuances and mathematical intricacies to consider. Let's delve into the details.

### 🚫 Discretization and Quaternion Constraints

- In the process of discretization, introducing extra parameters can violate the quaternion constraints. This can lead to the optimization process not converging.
  
### 🔄 Quaternion Parameterization during Optimization

- During the optimization process, instead of using a 4-parameter representation, we use a 3-parameter parameterization to represent the delta value of the quaternion. This helps in optimizing the objective function.
  
    $$
    \delta q = 
    \begin{bmatrix} 
    \cos\left(\frac{\Vert \phi \Vert}{2}\right) \\ 
    \frac{\phi}{\Vert \phi \Vert} \sin\left(\frac{\Vert \phi \Vert}{2}\right) 
    \end{bmatrix} \text{ (axis-angle representation)}
    $$
    
    Which can be approximated as:
    
    $$
    \delta q \approx 
    \begin{bmatrix} 
    \sqrt[]{1 - \phi^T \phi} \\ 
    \phi 
    \end{bmatrix} \text{ (vector part of quaternion)}
    $$
    
    And further approximated as:
    
    $$
    \delta q \approx \sqrt[]{1 + \phi^T \phi} 
    \begin{bmatrix} 
    1 \\ 
    \phi 
    \end{bmatrix} \text{ (Gibbs/Rodrigues vector)}
    $$

### 📈 Differentiating Quaternions

- When differentiating quaternions, especially with respect to the axis-angle vector, we need to consider a small disturbance:

    $$
    \delta q = 
    \begin{bmatrix} 
    \cos\left(\frac{\theta}{2}\right) \\ 
    a \sin\left(\frac{\theta}{2}\right) 
    \end{bmatrix} \approx 
    \begin{bmatrix} 
    1 \\ 
    \frac{1}{2} a \theta 
    \end{bmatrix} \approx 
    \begin{bmatrix} 
    1 \\ 
    \frac{1}{2} \phi 
    \end{bmatrix} = 
    \begin{bmatrix} 
    1 \\ 
    0 
    \end{bmatrix} + \frac{1}{2} 
    \begin{bmatrix} 
    0 \\ 
    \phi 
    \end{bmatrix}
    $$

- **Attitude Jacobian**: This Jacobian relates the change in the quaternion to the change in the axis-angle vector.

    $$
    q' = q * \delta q = L(q) \left( \begin{bmatrix} 1 \\ 0 \end{bmatrix} + \frac{1}{2} H \phi \right) = q + \frac{1}{2} L(q) H \phi
    $$
    
    Leading to the definition of the Attitude Jacobian:
    
    $$
    G(q) = \frac{\partial q}{\partial \phi} = \frac{1}{2} L(q) H \in \mathbb{R}^{4 \times 3}
    $$

- The 3-parameter representation can be converted to reflect the change in the 4-parameter quaternion.

- **Cost Function Gradient**: Given the gradient with respect to the quaternion as a 4x3 matrix, we can compute the gradient with respect to the 3-parameter representation.

    $$
    \frac{\partial J}{\partial \phi} = \frac{\partial J}{\partial q} \frac{\partial q}{\partial \phi} = \frac{\partial J}{\partial q} G(q)
    $$

- **Cost Function Hessian**:

    $$
    \nabla^2 J(q) = G^T(q) \frac{\partial^2 J}{\partial q^2} G(q) + I_3 \left(\frac{\partial J}{\partial q} q\right)
    $$

- **System Dynamic Gradient**: Mapping from \( f(q): \mathbb{H} \to \mathbb{H} \) to \( g(q(\phi)):\phi \to \phi \):

    $$
    \frac{\partial g}{\partial \phi} = \frac{\partial g}{\partial f} \frac{\partial f}{\partial q} \frac{\partial q}{\partial \phi} = \left[ G^T(f(q)) \frac{\partial f}{\partial q} G(q) \right] \in \mathbb{R}^{3 \times 3}
    $$

### 📝 Example: Wahba's Problem for Pose Estimation

- The objective function for Wahba's problem is:

    $$
    \min_q J(q) = \sum_{k=1}^m \Vert ^N x_k - Q(q) ^Bx_k \Vert_2^2 = \Vert r(q) \Vert_2^2
    $$

- The Jacobian for this problem is:

    $$
    r(q) = \begin{bmatrix} ^N x_1 - Q ^B x_1 \\ ^N x_2 - Q ^B x_2 \\ ...\\ ^N x_m - Q ^B x_m \end{bmatrix} \in \mathbb{R}^{3m \times 1}
    $$
    
    Leading to:
    
    $$
    \nabla_\phi r(q) = \frac{\partial r}{\partial q} G(q) \in \mathbb{R}^{3m \times 3}
    $$

- **Gaussian-Newton Method**:

    $$
    \min_x J(x) = \frac{1}{2} \Vert r(x) \Vert_2^2 = \frac{1}{2} r(x)^T r(x)
    $$
    
    With its derivatives:
    
    $$
    \frac{\partial J}{\partial x} = r(x)^T \frac{\partial r}{\partial x}
    $$
    
    And:
    
    $$
    \frac{\partial^2 J}{\partial x^2} = \left(\frac{\partial r}{\partial x}\right)^T \left(\frac{\partial r}{\partial x}\right) + (I \otimes r(x)^T) \frac{\partial^2 \text{vec}(r)}{\partial x^2}
    $$

- **Algorithm Overview**:

    The algorithm focuses on the gradient with respect to an intermediate variable but updates the final value.
    
    ```
    Initialize: q ← q0
    Repeat:
        1. Compute gradient: 
            \nabla r(q) = \frac{\partial r}{\partial q} G(q)
        2. Update phi:
            \phi = \left[ \left( \nab    abla r^T \nabla r \right)^{-1} \nabla r^T \right] r(q)
        3. Update quaternion:
            q ← q * \begin{bmatrix} \sqrt{1-\phi^T \phi} \\ \phi \end{bmatrix}
        4. Line search (if necessary) to ensure convergence.
    Until ||r(q)|| < tolerance
    ```

- **Note**: The solution might be \( q_{\text{true}} \) or \( -q_{\text{true}} \). It's essential to verify the correctness of the solution in the context of the problem.

### 📌 Key Takeaways

1. **Quaternion Constraints**: Introducing extra parameters during discretization can break quaternion constraints, leading to non-convergence in optimization.
2. **Parameterization**: Using a 3-parameter representation for the delta value of the quaternion can aid in optimization.
3. **Differentiation**: The gradient and Hessian of the cost function, as well as the system dynamic gradient, play crucial roles in the optimization process.
4. **Wahba's Problem**: This serves as a practical example of how quaternions can be optimized in the context of pose estimation.
5. **Algorithmic Approach**: The iterative approach to updating the quaternion ensures convergence to the optimal solution.

By understanding these intricacies, one can effectively optimize quaternions in various applications, ensuring accurate and efficient results.


# LQR with Quaternions 📐

## Introduction 📖

Linearizing a system with a quaternion state naively can lead to an uncontrollable linear system. To address this, we need to apply certain quaternion tricks.

## Problem Setup 🛠

### Linearization around the Reference Trajectory 📈

Given a reference trajectory $\bar{x}_k, \bar{u}_k$ for a discrete-time system $f(x_k, u_k)$, we can express the linearized system as:

$$
\begin{split}
\bar{x}_{k+1} + \Delta x_{k+1} &= f(\bar{x}_k + \Delta x_k, \bar{u}_k + \Delta u_k) \\
&\approx f(\bar{x}_k, \bar{u}_k) + A_k \Delta x_k + B_k \Delta u_k
\end{split}
$$

For the quaternion part of the state, we use the attitude Jacobian to convert $\Delta q \to \phi \in \mathbb{R}^3$:

$$
\begin{align*}
\begin{bmatrix}
\Delta x_{k+1}[1:3] \\
\phi_{k+1} \\
\Delta x_{k+1}[8:n]
\end{bmatrix} &= 
\begin{bmatrix}
I & & \\
& G(\bar{q}_{k+1}) & \\
& & I
\end{bmatrix}^T A_k 
\begin{bmatrix}
I & & \\
& G(\bar{q}_k) & \\
& & I
\end{bmatrix} + 
\begin{bmatrix}
I & & \\
& G(\bar{q}_{k+1}) & \\
& & I
\end{bmatrix}^T B_k \Delta u_k \\
\Delta \tilde x_{k+1} &= E(\bar{x}_{k+1}) A_k E(\bar{x}_{k}) \Delta x_k + E(\bar{x}_{k+1}) B_k \Delta u_k
\end{align*}
$$

With the "reduced" Jacobians $\tilde A_k, \tilde B_k$, we can compute the LQR gains:

$$
\tilde A_k = E(\bar{x}_{k+1}) A_k E(\bar{x}_{k}), \quad \tilde B_k = E(\bar{x}_{k+1}) B_k
$$

When running the controller, we first calculate $\Delta \tilde x$:

$$
\begin{align*}
\text{given } x_n, \Delta \tilde x_k &= 
\begin{bmatrix}
x_k[1:3] - \bar{x}_k[1:3] \\
\phi(L(\bar{q}_k)^T q_k) \\
x_k[8:n] - \bar{x}_k[8:n]
\end{bmatrix} \\
u_k &= \bar{u}_k - K_k \Delta \tilde x_k
\end{align*}
$$

### Computing Delta/Error Rotations 🔄

To compute the rotation from the body frame to the reference frame:

$$
\begin{align*}
^{N}{Q_k}^{B_k}, ^{N}{Q_{k+1}}^{B_{k+1}} &\Rightarrow ^{B_{k+1}}{}{Q_k}^{B_k} = (^{N}{}{Q_{k+1}}^{B_{k+1}}) (^{N}{}{Q_k}^{B_k})^{-1} =^{B_{k+1}}{}{Q_{k+1}}^N .^{N}{Q_k}^{B_k}) = Q_{k+1}^T Q_k \\
\Delta Q = \bar{Q}^T Q &\Leftrightarrow \Delta q = \bar{q}^* * q = L(\bar{q})^T q
\end{align*}
$$

## Example: 3D Quadrotor Control 🚁

### System Dynamics 📊

$$
\begin{align*}
T_i &= K_T u_i \\
M_i &= K_M u_i \\
u_i &\in \mathbb{R}^4
\end{align*}
$$

### State Representation 📌

For better control, it's advisable to operate in the body frame:

$$
x = 
\begin{bmatrix}
^{N}{}{r} \in \mathbb{R}^3 & \text{position in N frame} \\
q \in \mathbb{H} & \text{attitude } (B \to N)\\
^{B}{}{v} \in \mathbb{R}^3 & \text{linear velocity in B frame} \\
^{B}{}{\omega} \in \mathbb{R}^3 & \text{angular velocity in B frame}
\end{bmatrix}
$$

### Kinematics 🔄

$$
^{N}{\dot r} = ^{N}{}{v} = Q ^{B}{}{v} \\
\dot q = \frac{1}{2}q * \omega = \frac{1}{2} L(q) H ^B \omega
$$

### Translation Dynamics 🚀

$$
\begin{align*}
m ^{N}{}{\dot v} &= ^{N}{}{F} \\
^{N}{}{v} &= Q ^{B}{}{v} \Rightarrow ^{N}{}{\dot v} = \dot Q ^{B}{}{v} + Q ^{B}{}{\dot v} = Q \hat \omega ^{B}{}{v} + Q ^{B}{}{\dot v} \\
\Rightarrow ^{B}{}{\dot v} &= Q^T .^{N}{}{\dot v} - ^{B}{}{\omega} \times ^{B}{}{v} \\
\Rightarrow ^{B}{}{\dot v} &= \frac{1}{m} ^{B}{}{F} - ^{B}{}{\omega} \times ^{B}{}{v} \\
^{B}{}{F} &= Q^T 
\begin{bmatrix}
0 \\
0 \\
-mg
\end{bmatrix} + 
\begin{bmatrix}
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
K_T & K_T & K_T & K_T
\end{bmatrix}
\end{align*}
$$

### Rotation Dynamics 🌀

$$
\begin{align*}
J ^{B}{}{\dot \omega} + ^{B}{}{\omega} \times J ^{B}{}{\omega} &= ^{B}{}{\tau} \\
^{B}{}{\tau} &= 
\begin{bmatrix}
l K_T(u_2 - u_4) \\
l K_T(u_3 - u_1) \\
K_M(u_1 - u_2 + u_3 - u_4)
\end{bmatrix}
\end{align*}
$$
