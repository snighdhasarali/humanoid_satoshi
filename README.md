# humanoid_satoshi
Humanoid Control in PyBullet — Impedance, Admittance, IK, and HJB-Based Control

This project implements full-body humanoid control in PyBullet using multiple control strategies including impedance control, admittance control, inverse-kinematics–style arm motions, and an HJB-inspired optimal balance method.
The humanoid is simulated as a floating-base URDF model with active joint-torque control.

Overview

The objective of this project is to demonstrate fundamental and advanced control laws on a simulated humanoid model.
The controller executes:

Standing balance control

Walking gait generation

Admittance response to virtual external forces

Procedural IK-style arm movements

HJB-inspired optimal balance computation

All demonstrations run sequentially and automatically.

Control Theory Used

Below is a concise description of the control equations implemented in the simulation.

1. Impedance Control (Spring–Damper Model)

Joint torques are computed using a proportional–derivative (PD) law:

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

Variable	Meaning

𝑞
𝑖
∗
q
i
∗
	​

	Target joint angle

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

	Applied torque

This forms the core actuator model producing compliant, stable motion.

2. Admittance Control

In admittance control, external forces alter the commanded pose rather than creating torques directly.
The simplified formulation used is:

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

 is a virtual external force (e.g., wind, push)

𝛼
α is the admittance gain

Δ
𝑞
𝑖
Δq
i
	​

 is the displacement applied to the target pose

This allows the humanoid to react mechanically to disturbances.

3. Balance Control Using Base Orientation

Base roll and pitch, obtained from the floating base, are used to modify the ankle and hip joints to maintain upright posture.

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

This acts as a basic whole-body balance controller.

4. HJB-Inspired Optimal Balance Control

A simple approximation of a value function is used:

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

A balance gain that increases with tilt is computed as:

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

The ankle and hip corrective torques are then scaled accordingly:

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

This simulates an optimal feedback controller where control effort increases with estimated cost.

Humanoid Structure (Joint Overview)

The humanoid used is the standard PyBullet URDF model:

       Head
        |
      Torso
     /     \
  Shoulder Shoulder
     |        |
   Elbow     Elbow
      \
      Hips
     /   \
   Knee  Knee
     |      |
   Ankle   Ankle


All joints are automatically detected, and default PyBullet motors are disabled so that custom torque control can be applied.

Demonstrations Implemented

The simulation runs the following demonstrations sequentially:

1. Standing Balance

The humanoid maintains upright posture using ankle and hip corrections combined with impedance joint control.

2. Walking Motion

A periodic gait is generated using sinusoidal hip and knee profiles.
Arm swing is added for natural movement.
Balance control runs concurrently.

3. Admittance Response

The humanoid responds to time-varying virtual external forces (e.g., simulated wind or pushes).
The controller modifies the target pose according to the admittance law.

4. Arm Movement (IK-Style Pattern Generation)

Procedural periodic shoulder and elbow motions generate expressive arm movements resembling IK-driven gestures.

5. HJB-Inspired Optimal Balance

A tilt-based value function is computed, and corrective control is increased during large disturbances.

How to Run the Simulation
Requirements

Install the required Python packages:

pip install pybullet numpy

Running the Program

Execute the main script:

python3 humanoid_control.py


A PyBullet GUI window will launch and run all demonstrations automatically.

File Structure
HumanoidControl/
│
├── humanoid_control.py   # Main simulation file
├── README.md             # Project documentation
└── recordings/           # Optional: video output directory

Code Summary

The MovingHumanoid class performs the following:

Initializes PyBullet environment and loads a floating-base humanoid

Reads joint information and limits

Disables default velocity motors

Defines standing, walking, admittance, IK-style, and HJB-inspired controllers

Executes each demonstration in sequence

Provides a clean disconnect from the physics engine

The controller design is modular, allowing further expansion into areas such as LQR, MPC, RL, or full HJB solutions.
