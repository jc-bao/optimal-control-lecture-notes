## Lecture 16 Contact Dynamics

Contact dynamics is a crucial aspect of robotics and physics simulations, especially when dealing with the sudden change of dynamic variables during contact events. This section delves into the methods to handle such dynamics.

### Dealing with Contact

There are primarily two methods to handle contact dynamics:

1. **Hybrid Formulation**:
   - **Procedure**:
     1. Integrate the system dynamics.
     2. Check the guard function to detect contact.
     3. If contact is detected, apply a jump mapping.
     4. Continue integration post-contact.
   - **Advantages**:
     - Relatively easy to implement using standard algorithms.
     - Can be integrated by simply adding additional constraints or optimization variables during contact.
     - Has been successfully applied in locomotion simulations.
   - **Disadvantages**:
     - Requires a pre-specified contact mode sequence, which dictates which part of the robot is in contact at each time step. This can be problematic in scenarios with rich contact dynamics.

2. **Time-Stepping / Contact-Implicit Formulation**:
   - **Procedure**: Formulate contact as constraints and solve a constrained optimization problem at each timestep.
   - **Advantages**:
     - Doesn't require a pre-specified contact mode sequence.
   - **Disadvantages**:
     - The optimization problem becomes significantly more challenging.

### Example: Falling Brick

Consider a brick falling and making contact with the ground.

1. **Time-Stepping Method**:

   - **Equations**:

     - Mapping position to distance to the ground:
       $$
       \phi(q) = J [q_x \; q_y]^T
       $$
       where \( J = [0 \; 1] \).

     - Dynamics with contact force:
       $$
       m\left(\frac{v_{k+1}-v_k}{h}\right) = -mg + J^T\lambda_k
       $$

     - Backward Euler integration:
       $$
       p_{k+1} = p_k + hv_{k+1}
       $$

     - Contact constraints:
       $$
       \begin{align*}
       \phi(q_{k+1}) & > 0 \implies p[1] > 0 \\
       \lambda_k & \geq 0 \\
       \phi(q_{k+1})\lambda_k & = 0
       \end{align*}
       $$

     - Converted QP problem (derived from KKT conditions):
       $$
       \begin{align*}
       \min_{V_{k+1}} \ &0.5mV_{k+1}^TV_{k+1} + mV_{k+1}^T(hg - V_k) \\
       \text{s.t.} \ &J(p_k + hV_{k+1}) = 0
       \end{align*}
       $$

   - **Limitations**:

     - Doesn't solve for the exact impact time.
     - Contact forces are explicitly computed.
     - Doesn't generalize to higher-order integration methods like RK-4, necessitating smaller time steps.
     - Complementary conditions (boundary constraints) are non-smooth.
     - Widely used in simulation engines like PyBullet, DART, and Gazebo.

2. **Hybrid Method**:

   - **Equations**:

     - Smooth vector field:
       $$
       [\dot{q} \; \dot{v}]^T = [v \; -g ]^T
       $$

     - Guard function:
       $$
       \phi(x) \ge 0
       $$

     - Jump map:
       $$
       x' = g(x) = [q_x \; q_y \; v_x \; 0] ^T
       $$

   - **Procedure**:

     ```
     while t < t_final:
         if phi(x) ≥ 0:
             x_dot = f(x)
         else if phi = 0:
             x' = g(x)
     end
     ```

   - **Advantages**:

     - Solves for the exact impact time.
     - Can utilize high-accuracy integrators.

   - **Disadvantages**:

     - Doesn't compute contact forces.

   - **Applications**:

     - Widely used in trajectory optimization tools and model predictive control (MPC).

   - **Insight**: If the impact time is known in advance, each non-constrained trajectory can be optimized separately.

### Final Thoughts

Both the time-stepping and hybrid methods are widely used in the field of robotics and simulations. The choice between them often depends on the specific requirements of the task at hand.

## Hybrid Trajectory Optimization for Legged Systems

![Hybrid Trajectory Optimization Diagram](CMU16-745%20Optimal%20Control%20b4017ef3591745c2b0f29777c17a99ff/Untitled%207.png)

### State Representation

The state of the system is represented by the vector \( x \), which consists of:

- \( r_b \): Body position
- \( r_f \): Foot position
- \( v_b \): Body velocity
- \( v_f \): Foot velocity

Mathematically, the state is given by:

$$
x = \begin{bmatrix}
r_b \\
r_f \\
v_b \\
v_f 
\end{bmatrix} \in \mathbb{R}^8
$$

### Control Inputs

The control inputs for the system are represented by the vector \( u \), which consists of:

- \( F \): Force applied
- \( \tau \): Torque applied

Mathematically, the control input is given by:

$$
u = \begin{bmatrix}
F \\
\tau 
\end{bmatrix} \in \mathbb{R}^2
$$

### Jump Map

The jump map represents the transition of the state after a specific event, such as a foot contact. It is represented by:

$$
x' = g_{21}(x) = \begin{bmatrix}
r_b \\
f_f \\
v_b \\
0 
\end{bmatrix}
$$

By pre-specifying the contact time step, we can optimize the trajectory accordingly:

![Trajectory Optimization Diagram](CMU16-745%20Optimal%20Control%20b4017ef3591745c2b0f29777c17a99ff/Untitled%208.png)

## Reasoning about Frictions

When considering trajectory optimization, it's essential to account for the limitations imposed by friction. The frictional forces can be represented by:

$$
\begin{align*}
&||b||_2 \leq \mu n \\
&n \in \mathbb{R}_+ : \text{normal force} \\
&b \in \mathbb{R}^2 : \text{friction force}
\end{align*}
$$

However, the bottom of the friction cone is non-differentiable. To address this, we often linearize the constraints, leading to the concept of the friction pyramid:

$$
\begin{align*}
e^T d &\leq \mu n \ \ \ d \in \mathbb{R}^4 \ \ e = [1, 1, 1, 1]^T \\
d &\geq 0 \\
b &= [I \ -I] d
\end{align*}
$$

### Notes:

- If we want the system to slip, we need to introduce an additional mode.
- The friction pyramid is a coarse approximation and may not capture all the nuances of real-world frictional interactions.