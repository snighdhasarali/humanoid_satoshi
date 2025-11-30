humanoid_satoshi
Humanoid Control in PyBullet — Impedance, Admittance, IK, and HJB-Based Control
![20251201_000639](https://github.com/user-attachments/assets/502c36d2-133c-4c1e-8830-ec2f3e8fc686)


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

<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/4eb251f2-4b27-4323-ac78-785330fe04c6" />


3. impedance control
   <img width="720" height="235" alt="Screenshot (546)" src="https://github.com/user-attachments/assets/2bfbc3b3-ad17-4f15-812d-91d9477369c6" />

4. Inverse kinematics
   <img width="354" height="195" alt="Screenshot (547)" src="https://github.com/user-attachments/assets/93cec7cc-6173-43b6-8507-e28ef99c0867" />




6. HJB-Inspired Optimal Balance Control

<img width="672" height="539" alt="Screenshot (543)" src="https://github.com/user-attachments/assets/88d74139-850a-4edd-a1ac-da5fe7cee84a" />


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

