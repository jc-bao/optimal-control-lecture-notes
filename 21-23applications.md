# 🚀 Real-robot Applications

## 🌌 Land a Space Ship

### 📐 Convex Relaxation

Convex relaxation is a technique used to simplify complex optimization problems. The primary idea is to replace non-convex constraints with a larger convex one. This approach is particularly useful when dealing with problems that have non-linear objectives and constraints. A tight relaxation implies that for some specific problems, the solution to the relaxed version can provide the answer to the original problem.

- **Rocket Soft-landing Problem**: The goal is to land a rocket from an initial position \( x_0 \) to a final position \( z_f = 0 \) with a final velocity \( v_f = 0 \). The objective is to minimize fuel consumption and landing error. The constraints include thrust limits and safety constraints.

- **Full-stack Description**: 
    - **Position Controller**: Uses a point-mass model to reason about safety, thrust, fuel constraints, and generate acceleration. It operates at approximately 1Hz.
    - **Attitude Controller**: Deals with the rocket's attitude, flexible models, fluid slosh, and generates thrust and gimbal commands. It operates at approximately 10Hz.
    - **Rocket**: The final component that receives commands from the controllers.

- **Rocket Dynamics**: 
    - **Fuel Consumption**: Due to fuel consumption, the rocket's mass can change by 2-5x.
    - **Fluid Slosh**: This introduces high nonlinearity to the system and can be modeled as a pendulum.
    - **Flexible Models**: Since the rocket is lightweight, it's not stiff and has low-frequency bending modes. This requires the use of notch filters in the attitude controller.
    - **Aerodynamic Force**: Velocity constraints can address this.
    - **Attitude Controller**: Linear robust control is sufficient.

- **Convex Relaxation for Thrust**:

    Original (non-convex) thrust constraint:
    $$
    T_\text{min} \le ||T|| \le T_\text{max}
    $$
    
    Thrust angle constraints:
    $$
    n^T T/||T|| \le \cos(\theta_\text{max})
    $$
    
    Relaxed (and can be proved to be tight):
    $$
    \Gamma = ||T|| \gets \text{relaxed to} \; ||T|| \le \Gamma \\
    T_\text{min} \le \Gamma \le T_\text{max} \\
    n^T T \le \Gamma \cos(\theta_\text{max})
    $$

## 🤖 How to Walk a Legged Robot

Walking robots have evolved significantly over the years. The industry initially approached it from a quasi-stationary manipulation perspective, while academia focused on floating-based dynamics. The past two decades have seen advancements in both mechanical design and control techniques.

- **Full State of a Legged Robot**:
    - **State Estimator**: Uses sensors like joint encoders, IMUs, contact forces, and vision/GPS to estimate the robot's state.
    - **Gait / Footstep Planner**: Plans the robot's gait and foot-swing gestures.
    - **Body MPC Controller**: Treats the robot as a single rigid body and calculates desired joint forces and torques.
    - **Joint Controller**: Generates the desired force for each joint.

- **Body Dynamics**:
    The robot's body dynamics can be represented as:
    $$
    \underbrace{M(q)}_\text{mass matrix} \dot{v}+\underbrace{C(q,v)}_\text{Dynamic Bias Potential energy related} =\underbrace{B(q)}_\text{Input Jacobian}u  + \underbrace{J(q)^T}_\text{contact jacobian} \underbrace{f}_\text{ contact force} \\
    \phi(q) \ge 0 \gets \text{signed distance function}\\
    ||f^{2:3}|| \le \mu f' \gets \text{ friction cone}
    $$

- **Single-rigid-body / Centroidal Dynamics**:
    To simplify the dynamics, we can use a lumped single-rigid-body model for the whole body:
    $$
    m \dot{v} = -mg + \sum f_i \\
    J \dot{\omega} + \omega \times J\omega = \sum r_i \times f_i
    $$

## 🚗 Autonomous Driving

Autonomous driving involves a complex interplay of perception, planning, and control. The full stack for autonomous driving includes perception, high-level planning, path planning, and an MPC controller.

- **Dynamics**: There are multiple models to choose from, including the bicycle model, kinematic bicycle model, dynamic bicycle model, and double track model. The choice depends on the specific requirements and scenarios.

- **The Frozen Robot Problem**: This problem arises when we want the MPC controller to reason about coupled behaviors with other drivers and consider other cars' reactions to ours.

- **Game-theoretic Trajectory**: One approach is to assume that other cars are also solving a trajectory optimization problem. This leads to joint optimization for all cars, with one solution being the Nash Equilibrium.