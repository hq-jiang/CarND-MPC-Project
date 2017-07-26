## Model Predictive Control

In this project we are examining a more advanced control algorithm. It is called
model predictive control (mpc) and it is a really powerful control strategy, which
is 'optimal' in the sense that it minimized the cost function. This cost function
is defined by ourselves, so we can't escape the tuning process. But the cost parameters are very intuitive to handle.

Based on: https://github.com/udacity/CarND-MPC-Project  
Also refer to this link for installation

##### The Kinematic Model

We are using a kinematic model for our prediction, which works just fine for
a simulator.

The model has 6 states we need to keep track of, 4 are from the kinematics
and the other 2 are for the error states. As input we have the steering angle
`delta` and the throttle `a`.
* x, y: positions
* psi:  angle
* v:    velocity
* cte:  cross track error
* epsi: angular error
* delta: steering angle
* a: throttle

The have the following relationship:
```
x1 = x0 + v0*cos(psi0)*dt
y1 = y0 + v0*sin(psi0)*dt
psi1 = psi0 - v0/Lf * delta *dt
v1 = v0 + a * dt
cte1 = poly(x0) - y0 + v0 * sin(epsi0) * dt;
epsi1 = psi0 - atan2(psi_dest) - v0/Lf * delta *dt;
```
with the constraints:
```
25 <= delta <= 25 (in degree)
-1 <= a <= 1
```

##### Lane equation
For the reference trajectory we have a (somewhat) fixed set of points given
by the simulator which we want to follow. For a medium distance foresight, we
we choose a 3rd order polynomial which matches our reference trajectory smoothly.
In order to fit the polynomial reliably (numerically stable) we transform the reference points to vehicle coordinates, so that the polynomial is always close to
horizontal.

```
shift_x = ptsx[i] - px;
shift_y = ptsy[i] - py;
ptsx[i] = (shift_x * cos(0-psi) - shift_y*sin(0-psi));
ptsy[i] = (shift_x * sin(0-psi) + shift_y*cos(0-psi));
```

##### Optimization
We use the Ipopt library for optimizing our cost function and CppAd for automatic
differentiation. Our goal is to minimize our cost with respect to our current and future states through a set consecutive inputs (`delta` and `a`) given the kinematic
constraints.
As mentioned we define the cost ourselves and have quite a lot of
freedom in our design. Of course, we want to minimize our cross track error `cte` and also our angle relative `epsi` to the reference trajectory. But we also want to have a smooth steering angle `delta` and want to overall avoid big steering angles. How important is the velocity `v` for us? All of these decisions can be modeled mathematically as a simple squared error with additional tunable weights.  
There are also two other variables. The timestep `dt` and the number of timesteps
`N`. Generally we want to look some distance ahead to take curves into account. If
`dt*N` is to low we end up to something similar to a PID control. If `N` is big we look farther ahead, but if it is to high, we not only have a computational overhead but our polynomial would also fail to fit. Also since we are dealing with latency and inaccuracies, the more we look into the future, the worse our prediction gets. This in consequence has an impact on our current state, which might lead to errors. In my case the car osciallated a lot after increasing `N`, but it might also be due to the simulator not having a fixed reference trajectory.

##### Latency
At last we want to deal with latency in our control scheme. The advantage of mpc
is that we can do that quite easily by updating our model with a latency time step
before feeding it into the solver. The result is our control input in the future,
after letting the model transitioning one time step without input.
Since the default latency is only 0.1s it doesn't affect us very much. But after taking latency into account oscillation of the car decreased.


##### Future works
- Increase speed
