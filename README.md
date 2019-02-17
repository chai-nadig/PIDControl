# PID Control
PID controller to control the steering angle of a self driving car

---

## P - Controller
- A controller with only a P gain oscillates a lot.
- The amplitude of these oscillations are directly proportional to the gain.
- Here's a video of the self driving car steering only on P - Controller.
![P - Controller](p_control.gif)

## PD - Controller
- A P - Controller with a D gain is a PD - Controller.
- The oscillations generated by the P gain are dampened by the D gain.
- There's a maximum dampening that can be achieved using the D gain.
- Here's a video of the self driving car steering on a PD - Controller.
![PD - Controller](pd_control.gif)
