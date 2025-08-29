Robot Control​ refers to the algorithms and methods that enable robots to execute desired motions and tasks. In this study, simple joint-space control strategies were employed to track pre-planned trajectories.

#############################################PID.slx
In this study, a combined ​feedforward + feedback control​ strategy was adopted.
​Test 1: Verified the ​stability and disturbance rejection performance​ of a PID controller.
​Tests 2/3/4: Implemented ​PI control, transforming the system into a standard second-order system. Compared error responses under ​critical damping, ​over-damping, and ​under-damping​ conditions.
​Tests 5/6: Introduced ​derivative (D) control​ to compare error responses between PI and PID configurations, highlighting the role of derivative action.
​Test 7: Used ​proportional (P) control only, reducing the system to a first-order dynamic response.

#############################################LQR.m
This program implements ​optimal control (LQR)​. Since the desired joint angle r(t) is a time-varying function, the problem becomes ​nonlinear. To solve the LQR optimization, ​gradient descent​ is employed. By tuning the weighting matrices Q (state cost) and R (control effort cost), different control performances can be achieved.

#############################################MPC.m
Similar to the solution approach of LQR, ​MPC optimizes control actions over a finite time horizon at each time step, but only applies the ​first step​ of the optimized control sequence as the actual input for the current moment.

#############################################Sliding.slx
In this program, Sliding Mode Control (SMC), High-Gain Control, and High-Frequency Control​ were implemented and compared.
Tests ​1//3​: Implemented ​sliding mode control (SMC)​.By adjusting the parameters ​c​ and ​ρ, different control performances were achieved.However, due to the ​sign(e)​​ function, the control signal requires ​high-frequency switching, demanding higher hardware capabilities.
Test ​4​: High-Gain Control, where large control inputs drive rapid error convergence, but require the actuator to deliver ​instantaneously high control signals.
Tests ​5/6/7:​ High-Frequency Control, which ​sacrifices error convergence speed​ for ​reduced fluctuations in control input, resulting in smoother actuator operation.
