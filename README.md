# Project Overview

![high-level control diagram](media/diagram/control_diagram.png)
*Figure 1: High-level control/estimation loop. The nonlinear planar quadrotor + suspended payload model is simulated (environment). The model is linearized about an equilibrium (x̄, ū) to compute continuous-time LQR gains (A,B → solve ARE → K). The controller applies u = ū + K (x̂ − x̄). An EKF fuses noisy measurements (x, z, θ_d) with the nonlinear model to estimate the full state x̂_k, which is fed back to the controller. Disturbances and sensor noise are injected at the environment (w, v)*
        

This project studies a planar quadrotor with a suspended payload under nonlinear dynamics and limited sensing. The system is modeled using first-principles dynamics and linearized around a hover equilibrium to design an LQR controller for trajectory tracking. An Extended Kalman Filter (EKF) estimates the full system state from a minimal set of realistic measurements, while disturbances and sensor noise are injected at the nonlinear plant level to evaluate stability, robustness, and controller tuning under non-ideal conditions.


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

# Results

## Overview
Goal of control is to minimize $\theta_p$ while be able to reach the final goal.

Q, R intuition

- $Q_x, Q_z$: penalize position error; higher → faster convergence, but too high → aggressive maneuvers and larger payload swing.
- $Q_{\dot x}, Q_{\dot z}$: penalize speed; higher → safer for payload (limits induced swing) and reduces overshoot.
- $Q_{θ_d}, Q_{\dot θ_d}$: keep drone attitude near horizontal; penalize large tilt and fast attitude changes to avoid motor saturation and structural stress.
- $Q_{θ_p}, Q_{\dot θ_p}$: highest priority if you want to protect the package. Penalize payload angle strongly (θ_p) and its rate to prevent oscillation.
- R (u₁,u₂): penalize actuator effort. Larger R → smoother, safer thrusts; smaller R → more aggressive control. Keep R so commanded thrusts remain within physical bounds (no motor saturation).

Other constants
- $m_d$ = 5 kg
- $m_p$ = 2 kg
- $M = m_d + m_p$
- $r_d$ = 0.5 m
- $l$ = 0.5 m
- $\underline x_{ref} = [5 , 0, 5 , 0, \frac \pi 2, 0, 0, 0]$
- $\underline u_{ref} = [\frac {Mg} 2, \frac {Mg} 2]$
- $K_i = \begin{bmatrix}  
0 & 0 & 5 & 0 & 0 & 0 & 0  & 0 \\
0 & 0 & 5 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}$
- Q_noise = np.eye(8) * 0.001
- $R_{noise_x} = 0.8^2$ (based on u-blox NEO-M8N)

- $R_{noise_z} = 0.1^2$ (based on Bosch BMP388 / BMP390)

- $R_{noise_{imu}} = 0.02^2$ (based on Bosch BMI088)
- $R_{noise} =
\begin{bmatrix} 0.8^2 &  &\\  
 & 0.1^2 &\\
 & & 0.02^2
 \end{bmatrix}$
 
- By 'The drone never settle', it means the drone does not come into defined settling state under 20 seconds
- Definition of settling state:

$$|x - x_{ref}| < 0.05 \wedge |z - z_{ref}| < 0.05 \wedge |\dot x| < 0.05 \wedge |\dot z| < 0.05 \wedge |\theta_p| < 10 \degree$$

*for u clipping*
- $u_{min} = [0.0, 0.0]$
- $u_{max} = [\frac {3M} 2, \frac {3M} 2]$

## Baseline

These are baseline Q and R
$$
Q_{baseline} = diag([25, 20, 25, 20, 15, 10, 30, 10]) \\
R_{baseline} = diag([0.5, 0.5])
$$

![baseline:true vs estimation](media/graphs/baseline/baseline_2vest.png)
*graphs comparing true values and estimation values of each variable*

![baseline:rmse of each variable](media/graphs/baseline/baseline_rmse.png)
*graph shows rmse of each variable*

![baseline:trajectory](media/graphs/baseline/baseline_traject.png)
*graph shows trajectory of the drone*

```
Max package swing (degree):  39.81150426563668 
Control Effort Motor 1: [4.37928378] 
Control Effort Motor 2: [4.43946955] 
Settling after: 5.258 seconds
```

## Failure case

As I mentioned, our goal is to minimize $\theta_p$ but overpenalize state $\theta_p$ leads to aggressive control. This pushes the system outside the validity region of the linearized model, resulting in instability despite correct linearization and estimation.

$$
Q_{fail} = diag([1, 0.5, 5, 1, 8, 1, 80, 10]) \\
R_{fail} = diag([0.5, 0.5])
$$

![fail:true vs estimation](media/graphs/fail/fail_2vest.png)
*graphs comparing true values and estimation values of each variable*

![fail:rmse of each variable](media/graphs/fail/fail_rmse.png)
*graph shows rmse of each variable*

![fail:trajectory](media/graphs/fail/fail_traject.png)
*graph shows trajectory of the drone*

```
Max package swing (degree):  179.99730329689325
Control Effort Motor 1: [39.21058755]
Control Effort Motor 2: [39.47316862]
The drone never settle
```

## Robustness test
*Note: I used the baseline Q and R in this section*
### Sinusoidal gust
In this section, I will be investigating the system when it is experienced by a sinusoidal gust:

$$F(t) = 0.1Mg \sin (2 \pi \cdot 1.5t)$$

![sinGust:true vs estimation](media/graphs/sinGust/sinGust_2vest.png)
*graphs comparing true values and estimation values of each variable*

![sinGust:rmse of each variable](media/graphs/sinGust/sinGust_rmse.png)
*graph shows rmse of each variable*

![sinGust:trajectory](media/graphs/sinGust/sinGust_traject.png)
*graph shows trajectory of the drone*

```
Max package swing (degree):  64.91850800077341
Control Effort Motor 1: [7.45489251]
Control Effort Motor 2: [7.46984197]
Settling after: 11.431000000000001 seconds
```

### Sensor noise increases
In this section, I will be investigating the system when it is experienced by abnormally high sensor noise. This is conducted by multiplying the default sensor noises with a constant (n).

#### n = 2
![sensorNoise:true vs estimation](media/graphs/sensorNoise/sensorNoise2_2vest.png)
*graphs comparing true values and estimation values of each variable*

![sensorNoise:rmse of each variable](media/graphs/sensorNoise/sensorNoise2_rmse.png)
*graph shows rmse of each variable*

![sensorNoise:trajectory](media/graphs/sensorNoise/sensorNoise2_traject.png)
*graph shows trajectory of the drone*

```
Max package swing (degree):  65.77895210530531
Control Effort Motor 1: [13.21638327]
Control Effort Motor 2: [13.2817721]
Settling after: 16.281 seconds
```

#### n = 3
![sensorNoise:true vs estimation](media/graphs/sensorNoise/sensorNoise3_2vest.png)
*graphs comparing true values and estimation values of each variable*

![sensorNoise:rmse of each variable](media/graphs/sensorNoise/sensorNoise3_rmse.png)
*graph shows rmse of each variable*

![sensorNoise:trajectory](media/graphs/sensorNoise/sensorNoise3_traject.png)
*graph shows trajectory of the drone*

```
Max package swing (degree):  66.97846057547555
Control Effort Motor 1: [18.20949201]
Control Effort Motor 2: [18.44967125]
Settling after: 16.342 seconds
```

#### n = 4
![sensorNoise:true vs estimation](media/graphs/sensorNoise/sensorNoise4_2vest.png)
*graphs comparing true values and estimation values of each variable*

![sensorNoise:rmse of each variable](media/graphs/sensorNoise/sensorNoise4_rmse.png)
*graph shows rmse of each variable*

![sensorNoise:trajectory](media/graphs/sensorNoise/sensorNoise4_traject.png)
*graph shows trajectory of the drone*

```
Max package swing (degree):  68.37264148916749
Control Effort Motor 1: [22.44804497]
Control Effort Motor 2: [22.81308394]
The drone never settle
```
