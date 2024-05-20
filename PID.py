import matplotlib.pyplot as plt

x_data: list[float] = []
sp_data: list[float] = []
y_data: list[float] = []

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

# Example program
pid = PID(Kp=1.0, Ki=0.1, Kd=0.5)
setpoint = 10.0
dt = 0.1
time = 0.0
current_value = 0.0

while time < 30.0:
    error = setpoint - current_value
    control_signal = pid.update(error, dt)
    current_value += control_signal * dt
    x_data.append(time)
    y_data.append(current_value)
    sp_data.append(setpoint)
    #print(f"time: {time:.2f}, setpoint: {setpoint:.2f}, current value: {current_value:.2f}, error: {error:.2f}, control signal: {control_signal:.2f}")
    time += dt

plt.plot(x_data, y_data, color='b', label = "valor")
plt.plot(x_data, sp_data, color='r', label = "setPoint")
plt.legend()
plt.show()