# Project Overview
This project studies a planar quadrotor slung suspended payload while maintaining direction towards its goal. The controller is designed via linearization around an equilibrium where drone is hovering and an Extended Kalman Filter to estimate full states from only certain realistically measurable states. In this repository, I will be investigating the effect of disturbance and limited sensors on stability and Q/R tuning. Furthermore, I will be exploring possibility of real world applications such as for the drone delivery system.

# System modelling
## Coordinates and assumptions
* Diagram ![diagram of the system](media/diagram/system_diagram.jpg)
* State 

$$q = [x, z, \theta_d, \theta_p] \\ \underline{x} = [q, \dot{q}] = [x, \dot{x}, z, \dot{z}, \theta_d, \dot{\theta_d}, \theta_p, \dot{\theta_p}]$$
*where $\theta_d$ is an angle of the drone and $\theta_p$ is an angle of the package* 

* Angle convention
	* $\theta_d$: measures clockwise perpendicular to the drone
	* $\theta_p$: measures counter-clockwise perpendicular to the drone
* The package is a point mass
* The rigid string is very light
* The drone is a 2 dimensional rod
* All disturbances and noises are Gaussian. 

## System dynamics
* $m_d, m_p$ are masses of the drone and the package, respectively. $l$ is a length of the rigid string from the drone to the package. $I_{cm}$ is the moment of inertia of the drone at the center of mass. $r$ is the length from CM of the drone to each propeller
* $[x, z]$ are vectors from origin to $CM in the $x-z$ coordinate
* Deriving kinetics energy $T$ and potential energy $U$

$$T = \frac 1 2 (m_d + m_p) (\dot{x}^2+\dot{z}^2) + m_pl\dot{\theta_p} (\dot{x}\cos{\theta_p} + \dot{z}\sin{\theta_p}) + \frac 1 2 m_p l ^2 \dot{\theta_p}^2 + \frac 1 2 I_{cm} \dot{\theta_d}^2$$
$$U = (m_p + m_d)gz - m_pgl\cos{\theta_p}$$
* Now, we use Euler–Lagrange equation

$$\frac d {dt} \frac {\partial} {\partial{\underline {\dot{q}}}} L - \frac {\partial} {\partial \underline {q}} L = Q$$

* From the Manipulator equation in q-space, $M(q)\ddot{q} + C(q,\dot{q})\dot{q} = \tau(q) + Bu$, we get

$$M = \begin{bmatrix}  
m_d+m_p & 0 & 0 & m_pl\cos \theta_p \\  
0 & m_d+m_p & 0  & m_pl\sin \theta_p \\
0 & 0 & I_{cm} & 0 \\
m_pl\cos \theta_p & m_pl\sin \theta_p & 0 & m_pl^2 \\
\end{bmatrix}, 
C(q, \dot{q}) = \begin{bmatrix}  
0 & 0 & 0 & -l\dot \theta_p\sin \theta_p \\  
0 & 0 & 0  & l\dot \theta_p\cos \theta_p \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
\end{bmatrix}$$

$$
\tau(q) = \begin{bmatrix}  
0 \\  
-(m_p+m_d)g \\
0 \\
-m_pgl\sin \theta_p
\end{bmatrix},
B = \begin{bmatrix}  
\cos \theta_d & \cos \theta_d\\  
\sin \theta_d & \sin \theta_d\\
r & -r \\
0 & 0
\end{bmatrix},
$$

* Thus, we can compute $\underline {\ddot q}$ with $\underline {\ddot q} = M^{-1}(\tau + Bu - C \underline {\dot q})$
* u is the input, in this case, thrusts created by each of the propellers


# Linearization
As we will be using LQR as a controller, it requires the system to be linear as it has to use $A$A and $B$B from the $\underline{\dot x} = A\underline{x} + B \underline{u}$ to compute $K$

To linearize the s we need to choose an equilibrium point, and linearize the system around that point. In our case, I would choose the equilibrium point where the drone is hovering horizontally and the packaging is hanging downward without any angle offset. Consequently, we need thrust equal to the weight of the drone to hover. Thus, our equilibrium state and input $\underline{u}$ is

$$\underline {\bar{x}} = [x_{ref}, 0, z_{ref}, 0, \frac \pi 2, 0, 0, 0]$$

$$\underline {\bar u} = [\frac 1 2 (m_d + m_p)g, \frac 1 2 (m_d + m_p)g]$$

*where $x_{ref}$ and $z_{ref}$ are final coordinates we want our drone to be*

Given $\underline{\dot x} = \underline{f}(\underline x, \underline u)$ where $\underline{f}(\underline x, \underline u)$ is a non-linear dynamics, we can approximate around an equilibrium point using Taylor's series

* $\Delta{\underline{x}}$ is a vector from an equilibrium point to a nearby point, as it is the nearby point, $\Delta{\underline{x}}$ is small, thus, $\Delta{\underline{x}}^2, \Delta{\underline{x}}^3, ...$ are incredibly small, so we can neglect them! Thus
$$\Delta\underline{\dot{x}} = \frac D {D\underline{x}} f(\bar{\underline{x}}, \bar{u}) \Delta{x}+ \frac D {Du} f(\bar{\underline{x}}, \bar{u}) \Delta{u}$$

* Note: $\frac D {D \underline{x}} f$ is a Jacobian matrix. 
* Thus $A = \frac D {D\underline{x}} f(\bar{\underline{x}}, \bar{u}), B = \frac D {Du} f(\bar{\underline{x}}, \bar{u})$, I can, now, use LQR, determine observability and controllability of the system!

# Controller
## Overview of the control, in full state feedback case.
![Diagram of LQR control](https://www.mathworks.com/discovery/optimal-control/_jcr_content/mainParsys/columns/daa10959-3b74-4985-b7e9-d12f3dee67b6/image_copy.adapt.full.medium.jpg/1765106504765.jpg "source: https://www.mathworks.com/discovery/optimal-control.html")*the image is from https://www.mathworks.com/discovery/optimal-control.html*

LQR is an optimal control. Unlike pole placement, you don't have to choose stable eigenvalues ($\lambda < 0$), and then solve for K, in LQR, we find the optimal K by choosing characteristic. Q and R are penalizing bad performance (error between desired state and the current state) and actuator effort (how aggressive we can command the actuator) respectively. The cost function is

$$J = \int_{0}^{\infty} (\underline{x}^{T}Q\underline{x} + u^{T}Ru) dt  $$
and our goal here is to find u such that minimize the cost function.
From math, $K$ is the optimal gain that results in minimizing the cost function J

*there is more mathematical detail on how to compute K, but I am not gonna write it out here as it so long, but I encourage you to look up the calculation detail from this MIT's Underactuated Robotics book [https://underactuated.mit.edu/lqr.html]*

* $Q$ is a diagonal matrix, and, in this case,

 $$Q = \begin{bmatrix}  
Qx &  & &  &  &  &  & \\  
 & Q_{\dot{x}}  & &  & & & &\\
 &  & Q_z &  &  &  &  & \\
 &  &  & Q_{\dot{z}} &  &  &  &  \\
 &  &  &  & Q_{\theta_d} &  &  &  \\
 &  &  &  &  & Q_{\dot {\theta_d}} &  &  \\
 &  &  &  &  &  & Q_{\theta_p} &  \\
 &  &  &  &  &  &  & Q_{\dot {\theta_p}} \\
\end{bmatrix}$$

* $R$ is also a diagonal matrix because we have 2 inputs, two propellers.

$$R = \begin{bmatrix}R_{u_1} &  \\  
 & R_{u_2}
 \end{bmatrix}$$

* $K_r$ is the optimal gain 1x8 matrix, 
$$K_r = \begin{bmatrix}  k_1 & ... & k_8 \end{bmatrix}$$ 

Furthermore, unlike the inverted pendulum system where the equilibrium control input is zero, a multirotor UAV requires a constant non-zero thrust to counteract gravity, even at a stable hover. Relying solely on the proportional feedback $\underline{\bar u}$ can lead to steady-state errors due to model uncertainties or unmodeled disturbances. To eliminate this, we introduce an integral term similar to what we do in PID.

$$K_i = \begin{bmatrix}  
k_x & k_{\dot{x}} & k_z & k_{\dot{z}} & k_{\theta_d} & k_{\dot{\theta_d}} & k_{\theta_p} & k_{\dot{\theta_p}} \\
k_x & k_{\dot{x}} & k_z & k_{\dot{z}} & k_{\theta_d} & k_{\dot{\theta_d}} & k_{\theta_p} & k_{\dot{\theta_p}} \end{bmatrix} $$

*Each row corresponding for each propeller*

It is worth to note that only the gains corresponding to the altitude ($K_z$) are non-zero; all other elements are set to zero. This is because gravity acts solely in the vertical axis, z.

However, in this project, $\underline{y} \ne \underline x$ but instead $\underline y = C\underline x$. So instead of straight up passing $\underline y$ into our controller like in the diagram, we must pass $\underline y$ into our observer first.

# Observer
## Overview of the observer
In this project, we assume to have access to 3 states out of total 8 states
1. Horizontal coordinate, $x$, via a GPS
2. Vertical coordinate, $z$, via an altimeter 
3. Drone tilt, $\theta_d$, via an IMU

*These  are minimum sensors on a real drone.*

Thus, here is our $C$ matrix

$$C = \begin{bmatrix}  
1 & 0 & 0 & 0 & 0 & 0 & 0  & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0\end{bmatrix} $$

Look it closely, you will see that each row represents each sensor!

And $\underline{y}$ is a measurable states vector, where $\underline{y} = C\underline{x}$

### Observability 
Before proceed anything further, we should look if available sensors are sufficient to estimate, aka. "know", every states or not. We can check this easily by looking at the rank of the matrix $\mathcal{O}$ where 

$$\mathcal{O} = [C, CA, CA^2, ..., CA^{n-1}]^T$$
*n is number of states in $x$, and, in this case, n = 8*

* If rank($\mathcal{O}$) = n, the system is observable, which means we can estimate full state $\underline{x}$ from $\underline{y}$

In our case, around equilibrium point, $\underline {\bar x}$, the system is observable with matrix $C$.

## Extended Kalman Filter (EKF)
In this project, due to nature of our non-linear system, we choose to use EKF as our observer instead of the traditional Kalman Filter(KF). EKF will estimate all 8 states, $\hat {\underline x}$.

Like KF, EKF consists with 2 phases; prediction and update. 

**Prediction**: predict the next state $\hat{\underline x}^-$ using **non-linear** physics model. 

$$\hat{\dot {\underline x}}_{k} = \underline{f} (\underline {\hat x}_{k-1}, \underline u)$$

$$\hat{\underline x}^{-}_{k} = \hat {\underline{x}}_{k-1} +\hat{\dot {\underline x}}_{k} \cdot dt$$

Because we did not measure yet, uncertainty, $P_k^-$, increases.

$$P_{k}^- = FP_{k-1}F^T + Q_{noise}$$

* *$Q_{noise}$ is the Process Noise Covariance, it is a hyperparameter we have to tune. It is to capture how inaccurate our model is. It is high if we think the environmental disturbance (e.g. gust) is high*

* *$F$ is a Jacobian matrix at current state $\underline {\hat x}$, i.e., $\frac D {D \underline x}f(\underline x, \underline u)|_{x =\underline {\hat x}_{k-1}}$*

**Update**: Measure $\underline y$ and correct the prediction
We get $\underline y$ from the sensors (This is happening in the sensors, not what we can control)

$$\underline y = C \underline{x} + noise$$

Thus, our prediction error is 

$$\Delta \underline y = \underline y - C\hat{\underline x}^{-}_{k} $$

And uncertainty of measurement from the sensors can compute with

$$S =CP_{k+1}^- C^T + R_{noise}$$
* *$R_{noise}$ is the sensors noise matrix, it often can be found in sensor sheet*

Now, we can compute $K_f$ or Kalman gain which tells us how much we should trust the sensors

$$K_f = P_{k+1}^-C^TS^{-1}$$

Finally, it is time to get $\hat {\underline x}_k$

$$\hat {\underline x}_k = \hat{\underline x}^{-}_{k} + K_f(\Delta {\underline y})$$



$\hat {\underline x}_k$ is what we will send to the controller!
