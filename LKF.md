# Simple Car Tracking with a Kalman Filter

This project has two MATLAB files:

- **SimpleCar.m** — makes a fake (simulated) car that moves around, and takes noisy sensor readings of its position.
- **SimpleCarLKF.m** — uses a **Linear Kalman Filter (LKF)** to clean up the noisy readings and estimate where the car really is.

Below is a simple explanation of what's happening and the math behind it.

---

## 1. The car's state

We describe the car with 4 numbers (the "state"):

```
x = [ px, vx, py, vy ]
```

- `px` = position in x
- `vx` = velocity in x
- `py` = position in y
- `vy` = velocity in y

## 2. How the car moves (SimpleCar.m)

Every small time step (`dt = 0.01 s`), the car's state updates using basic motion equations (position = old position + velocity × time, etc.). In matrix form:

```
x_k = F * x_(k-1) + G * u_k + noise
```

Where:

- **F** (state transition matrix) moves the state forward in time:

```
F = [ 1  dt  0   0
      0  1   0   0
      0  0   1   dt
      0  0   0   1 ]
```

- **G** (control matrix) applies the acceleration input `u_k = [ax; ay]`:

```
G = [ 0.5*dt^2   0
      dt         0
      0          0.5*dt^2
      0          dt ]
```

- **noise** is small random "process noise" added to make the simulation more realistic (things like tiny bumps in the road).

### The sensor

The sensor doesn't see velocity, only position, and it's noisy:

```
z_k = H * x_k + sensor_noise
```

- **H** (measurement matrix) picks out just the position values:

```
H = [ 1  0  0  0
      0  0  1  0 ]
```

- `sensor_noise` is random noise added to make the measurement imperfect, like a real GPS.

---

## 3. Cleaning up the noise (SimpleCarLKF.m) — The Kalman Filter

The Kalman Filter has two steps that repeat every time step: **Predict** and **Update**.

### Step A — Predict

Guess where the car is now, based on where it was before:

```
priori_x_k            = F * posterior_x_(k-1) + G * u_k
priori_x_k_covariance  = F * posterior_P_(k-1) * F' + Q
```

- `priori_x_k` = our best guess of the state *before* looking at the sensor
- `priori_x_k_covariance` (often called **P**) = how uncertain we are about that guess
- **Q** = process noise covariance matrix (how much we trust the motion model)

### Step B — Update

Now compare the guess to what the sensor actually says, and correct it:

```
y_k = z_k - H * priori_x_k                     (innovation: sensor vs. prediction)
S_k = H * priori_x_k_covariance * H' + R        (innovation covariance)
K_k = priori_x_k_covariance * H' * inv(S_k)     (Kalman Gain)

posterior_x_k            = priori_x_k + K_k * y_k
posterior_x_k_covariance = (I - K_k * H) * priori_x_k_covariance
```

- `y_k` = the difference between what the sensor says and what we predicted (the "surprise")
- **R** = sensor noise covariance matrix (how much we trust the sensor)
- **K_k** (Kalman Gain) = decides how much to trust the sensor vs. the prediction
  - If K_k is big → trust the sensor more
  - If K_k is small → trust the prediction more
- `posterior_x_k` = the final, corrected estimate of the state
- `posterior_x_k_covariance` = updated uncertainty after using the sensor info

This Predict → Update loop runs every time step, and the filter keeps getting a smoother, more accurate idea of where the car is, even though the sensor readings are noisy.

---

## 4. What the code shows you

- **SimpleCar.m** plots the true position/velocity of the car and the raw noisy sensor readings.
- **SimpleCarLKF.m** plots:
  - The filtered (cleaned-up) position estimate over time
  - The uncertainty (standard deviation) of that estimate over time
  - A bird's-eye view comparing: noisy sensor dots, the true path, and the Kalman filter's estimated path
  - RMSE (root-mean-square error) numbers comparing the sensor's error vs. the Kalman filter's error, to show how much the filter improves accuracy

## 5. In short

The Kalman filter is basically an educated guessing machine:
1. Predict where the car should be using physics.
2. Check the noisy sensor.
3. Blend the two together, trusting whichever one is more reliable at that moment.
4. Repeat, getting smarter over time.
