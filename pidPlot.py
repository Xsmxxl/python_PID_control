from time import sleep
from os import system
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PID:
    def __init__(self, Kp: float, Ki: float, Kd: float, Setpoint: int, antiWindup: bool = False):
        self.Kp: float = Kp
        self.Ki: float = Ki
        self.Kd: float = Kd
        self.setpoint: int = Setpoint
        self.dt: float = 0.1
        self.last_error: float = 0.0
        self.integral: float = 0.0
        self.antiWindup: bool = antiWindup
        self.error: float = 0.0
        self.output: float = 0.0

    def update(self, vc, LIMITE_MAX: float, LIMITE_MIN: float):
        self.error = self.setpoint - vc
        derivative = (self.error - self.last_error) / self.dt

        if abs(self.output) >= LIMITE_MAX and (((self.error >= 0.0) and (self.integral >= 0.0)) or ((self.error < 0.0) and (self.integral < 0.0))):
            if self.antiWindup:
                self.integral = self.integral
            else:
                self.integral += self.error * self.dt
        else:
            self.integral += self.error * self.dt

        self.output = self.output + (self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative) * self.dt
        self.last_error = self.error
        if self.output >= LIMITE_MAX:
            self.output = LIMITE_MAX
        if self.output <= LIMITE_MIN:
            self.output = LIMITE_MIN
        return (self.setpoint, self.output)

class AnimationPlot:
    def __init__(self):
        self.sp_data: list[float] = []
        self.y_data: list[float] = []
        self.t_data: list[float] = []
        self.signal: float = 0.0
        self.LIMITE_MAX: float = 4000
        self.LIMITE_MIN: float = 475
        self.setpoint: float = 0.0
        self.time: float = 0.1
        self.dt: float = 0.05

    def animate(self, _, ard, pid):
        arduinoData_string = ard.readline().decode('ascii')
        try:
            arduinoData_float = float(arduinoData_string)
            self.setpoint, self.signal = pid.update(arduinoData_float, self.LIMITE_MAX, self.LIMITE_MIN)
            self.y_data.append(arduinoData_float)
            self.sp_data.append(self.setpoint)
            self.t_data.append(self.time)
            if len(self.t_data) > 1000:
                self.t_data = self.t_data[1:]
                self.y_data = self.y_data[1:]
                self.sp_data = self.sp_data[1:]
            self.time += self.dt
            x = self.rpmToAnalog(int(self.signal))
            if x >= 255:
                x = 255
            elif x <= 55:
                x = 55
            ard.write(bytes(str(x), 'utf-8'))

            ax.clear()
            self.getPlotFormat()
            ax.plot(self.t_data, self.y_data, c='b', label=f"Lectura (RPM): {int(arduinoData_float)}")
            ax.plot(self.t_data, self.sp_data, c='r', label=f"Setpoint: {int(self.setpoint)}")
            ax.legend(loc='upper right')
        except:
            pass

    def rpmToAnalog(self, RPM: int):
        return int((RPM / self.LIMITE_MAX) * 255)

    def getPlotFormat(self):
        ax.set_ylim([0, (self.LIMITE_MAX + 250)])
        ax.set_title("Gráfica PID para controlar RPM")

def find_Kc_Pc(arduino):
    Kp = 0.1
    pid = PID(Kp=Kp, Ki=0.0, Kd=0.0, Setpoint=1500)
    plot_data = []
    osc_count = 0
    threshold = 100  # Threshold to determine stable oscillations

    while True:  # Run until oscillations are stable
        arduinoData_string = arduino.readline().decode('ascii')
        try:
            arduinoData_float = float(arduinoData_string)
            _, signal = pid.update(arduinoData_float, 4000, 475)
            plot_data.append(signal)
            if len(plot_data) > 10:
                if (max(plot_data[-10:]) - min(plot_data[-10:])) > threshold:
                    osc_count += 1
                    if osc_count > 5:  # Stable oscillations detected
                        break
                else:
                    osc_count = 0

            pid.Kp += 0.1  # Increment Kp

        except:
            pass

    Kc = pid.Kp
    Pc = 2 * realTimePlot.dt  # Rough estimation for Pc
    return Kc, Pc

fig = plt.figure('Control PID')
ax = fig.add_subplot(111)

realTimePlot = AnimationPlot()
arduino = serial.Serial("COM7", 115200)
sleep(2)
arduino.write(b'50')
system('cls')

# Encontrar Kc y Pc
Kc, Pc = find_Kc_Pc(arduino)
print(f"Ganancia crítica (Kc): {Kc}, Período crítico (Pc): {Pc}")

# Calcular los parámetros PID usando Ziegler-Nichols
Kp = 0.6 * Kc
Ti = Pc / 2
Td = Pc / 8
Ki = Kp / Ti
Kd = Kp * Td

print(f"PID Parameters: Kp={Kp}, Ki={Ki}, Kd={Kd}")

# Ajustes manuales (puedes modificarlos según sea necesario)
Kp_adjusted = Kp * 0.8
Ki_adjusted = Ki * 0.8
Kd_adjusted = Kd * 1.2

print(f"Ajustes manuales PID: Kp={Kp_adjusted}, Ki={Ki_adjusted}, Kd={Kd_adjusted}")

# Inicializar el controlador PID con los nuevos parámetros ajustados
pidInit = PID(Kp=Kp_adjusted, Ki=Ki_adjusted, Kd=Kd_adjusted, Setpoint=1500)

# Correr la animación con el PID ajustado
ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=100, fargs=(arduino, pidInit), interval=10)
plt.draw()
plt.show()

arduino.write(b'0')
sleep(0.1)
arduino.write(b'0')
arduino.close()
system('cls')
