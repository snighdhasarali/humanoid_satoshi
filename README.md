humanoid_satoshi
Humanoid Control in PyBullet — Impedance, Admittance, IK, and HJB-Based Control

This project implements full-body humanoid control in PyBullet using several control strategies, including impedance control, admittance control, procedural IK-style arm motions, and an HJB-inspired optimal balance method.
The humanoid model is simulated as a floating-base URDF system with joint-torque actuation.

Overview

The objective of this project is to demonstrate both fundamental and advanced humanoid control laws in a physics simulation environment.
The controller performs the following:

Standing balance control

Walking gait generation

Admittance-based reaction to virtual external forces

Procedural inverse-kinematics–style arm motion

HJB-inspired optimal balance control

All demonstrations run sequentially.

Control Theory

Below are the main control equations implemented in the system.

1. Impedance Control (PD Model)

Joint torques are computed using a spring–damper formulation:

𝜏
𝑖
=
𝐾
𝑝
(
𝑞
𝑖
∗
−
𝑞
𝑖
)
−
𝐾
𝑑
𝑞
˙
𝑖
τ
i
	​

=K
p
	​

(q
i
∗
	​

−q
i
	​

)−K
d
	​

q
˙
	​

i
	​


Where:

Symbol	Meaning

𝑞
𝑖
∗
q
i
∗
	​

	Desired joint angle

𝑞
𝑖
q
i
	​

	Actual joint angle

𝑞
˙
𝑖
q
˙
	​

i
	​

	Joint velocity

𝐾
𝑝
K
p
	​

	Stiffness gain

𝐾
𝑑
K
d
	​

	Damping gain

𝜏
𝑖
τ
i
	​

	Torque applied to joint 
𝑖
i

This forms the basic compliant control law for the humanoid.

2. Admittance Control

In admittance control, motion results from applied forces. The simplified form used is:

Δ
𝑞
𝑖
=
𝛼
𝐹
𝑖
Δ
𝑡
Δq
i
	​

=αF
i
	​

Δt

Where:

𝐹
𝑖
F
i
	​

: Virtual external force (push, wind, etc.)

𝛼
α: Admittance gain

Δ
𝑞
𝑖
Δq
i
	​

: Adjustment added to target joint position

This produces physically realistic reactions to disturbances.

3. Balance Control Using Base Orientation

The humanoid’s floating base roll and pitch angles are used to modify hip and ankle joint targets:

𝑞
ankle
∗
=
𝑞
stand
−
0.5
⋅
pitch
−
0.3
⋅
roll
q
ankle
∗
	​

=q
stand
	​

−0.5⋅pitch−0.3⋅roll
𝑞
hip
∗
=
𝑞
stand
+
0.2
⋅
pitch
q
hip
∗
	​

=q
stand
	​

+0.2⋅pitch

This forms a simple but effective closed-loop balance controller.

4. HJB-Inspired Optimal Balance Control

A value function representing tilt magnitude is defined as:

𝑉
=
∣
roll
∣
+
∣
pitch
∣
V=∣roll∣+∣pitch∣

The balance gain increases with tilt:

𝐾
bal
=
1
+
2
𝑉
K
bal
	​

=1+2V

Corrective ankle control is then scaled as:

𝑞
ankle
∗
=
𝑞
stand
−
𝐾
bal
⋅
0.5
⋅
pitch
q
ankle
∗
	​

=q
stand
	​

−K
bal
	​

⋅0.5⋅pitch

This imitates an optimal feedback controller derived from Hamilton–Jacobi–Bellman principles.

Humanoid Structure

The PyBullet humanoid URDF contains the following conceptual structure:

         Head
          |
        Torso
     /         \
  Shoulder   Shoulder
      |          |
    Elbow      Elbow
        \
        Hips
      /      \
   Knee      Knee
     |          |
   Ankle      Ankle


All joints are read at runtime, and default PyBullet motors are disabled so that custom torque commands can be applied.

Demonstrations

The simulation executes these demonstrations in sequence:

1. Standing Balance

Maintains upright posture using ankle and hip corrections plus impedance control.

2. Walking Motion

Generates sinusoidal hip and knee trajectories, with natural arm swing.
Balance control runs simultaneously.

3. Admittance Response

Applies virtual external forces that cause the humanoid to sway or shift according to the admittance law.

4. Arm Movement (IK-Style Motion)

Shoulder and elbow joints move procedurally using periodic functions, approximating IK-driven gestures.

5. HJB-Inspired Balance

Implements tilt-dependent corrective control using the value function and scaled gain.

How to Run the Simulation
Requirements

Install the necessary packages:

pip install pybullet numpy

Running
python3 humanoid_control.py


The PyBullet GUI will open and automatically run the demonstration sequence.

File Structure
HumanoidControl/
│
├── humanoid_control.py      # Main simulation script
├── README.md                # Documentation
└── recordings/              # Optional: mp4 recordings directory

Code Summary

The MovingHumanoid class is responsible for:

Initializing the PyBullet environment

Loading the floating-base humanoid model

Detecting joints, limits, and disabling default motors

Implementing impedance, admittance, IK-style, and HJB-inspired control laws

Running all demonstrations automatically

Closing the simulation cleanly

The design is modular and can be extended with control techniques such as LQR, MPC, reinforcement learning, or full HJB solvers.

