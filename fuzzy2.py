import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from time import sleep
from os import system
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class FuzzyPID:
    def __init__(self, Setpoint):
        self.setpoint = Setpoint
        self.last_error: float = 0.0

        # Definición de las variables difusas
        self.error = ctrl.Antecedent(np.arange(-2000, 2000, 1), 'error')
        self.delta_error = ctrl.Antecedent(np.arange(-1000, 1000, 1), 'delta_error')
        self.output = ctrl.Consequent(np.arange(-255, 255, 1), 'output')

        # Funciones de pertenencia
        self.error['NB'] = fuzz.trapmf(self.error.universe, [-2000, -2000, -1000, -500])
        self.error['NM'] = fuzz.trimf(self.error.universe, [-1000, -500, 0])
        self.error['NS'] = fuzz.trimf(self.error.universe, [-500, 0, 500])
        self.error['ZE'] = fuzz.trimf(self.error.universe, [-500, 0, 500])
        self.error['PS'] = fuzz.trimf(self.error.universe, [0, 500, 1000])
        self.error['PM'] = fuzz.trimf(self.error.universe, [500, 1000, 2000])
        self.error['PB'] = fuzz.trapmf(self.error.universe, [1000, 1500, 2000, 2000])

        self.delta_error['NB'] = fuzz.trapmf(self.delta_error.universe, [-1000, -1000, -500, -250])
        self.delta_error['NM'] = fuzz.trimf(self.delta_error.universe, [-500, -250, 0])
        self.delta_error['NS'] = fuzz.trimf(self.delta_error.universe, [-250, 0, 250])
        self.delta_error['ZE'] = fuzz.trimf(self.delta_error.universe, [-250, 0, 250])
        self.delta_error['PS'] = fuzz.trimf(self.delta_error.universe, [0, 250, 500])
        self.delta_error['PM'] = fuzz.trimf(self.delta_error.universe, [250, 500, 1000])
        self.delta_error['PB'] = fuzz.trapmf(self.delta_error.universe, [500, 750, 1000, 1000])

        self.output['NB'] = fuzz.trapmf(self.output.universe, [-255, -255, -170, -85])
        self.output['NM'] = fuzz.trimf(self.output.universe, [-170, -85, 0])
        self.output['NS'] = fuzz.trimf(self.output.universe, [-85, 0, 85])
        self.output['ZE'] = fuzz.trimf(self.output.universe, [-85, 0, 85])
        self.output['PS'] = fuzz.trimf(self.output.universe, [0, 85, 170])
        self.output['PM'] = fuzz.trimf(self.output.universe, [85, 170, 255])
        self.output['PB'] = fuzz.trapmf(self.output.universe, [170, 212, 255, 255])

        # Definición de las reglas difusas
        self.rules = [
            ctrl.Rule(self.error['NB'] & self.delta_error['NB'], self.output['NB']),
            ctrl.Rule(self.error['NB'] & self.delta_error['NM'], self.output['NB']),
            ctrl.Rule(self.error['NB'] & self.delta_error['NS'], self.output['NB']),
            ctrl.Rule(self.error['NB'] & self.delta_error['ZE'], self.output['NB']),
            ctrl.Rule(self.error['NB'] & self.delta_error['PS'], self.output['NM']),
            ctrl.Rule(self.error['NB'] & self.delta_error['PM'], self.output['NM']),
            ctrl.Rule(self.error['NB'] & self.delta_error['PB'], self.output['NS']),

            ctrl.Rule(self.error['NM'] & self.delta_error['NB'], self.output['NB']),
            ctrl.Rule(self.error['NM'] & self.delta_error['NM'], self.output['NB']),
            ctrl.Rule(self.error['NM'] & self.delta_error['NS'], self.output['NB']),
            ctrl.Rule(self.error['NM'] & self.delta_error['ZE'], self.output['NM']),
            ctrl.Rule(self.error['NM'] & self.delta_error['PS'], self.output['NM']),
            ctrl.Rule(self.error['NM'] & self.delta_error['PM'], self.output['NS']),
            ctrl.Rule(self.error['NM'] & self.delta_error['PB'], self.output['NS']),

            ctrl.Rule(self.error['NS'] & self.delta_error['NB'], self.output['NB']),
            ctrl.Rule(self.error['NS'] & self.delta_error['NM'], self.output['NB']),
            ctrl.Rule(self.error['NS'] & self.delta_error['NS'], self.output['NM']),
            ctrl.Rule(self.error['NS'] & self.delta_error['ZE'], self.output['NM']),
            ctrl.Rule(self.error['NS'] & self.delta_error['PS'], self.output['NS']),
            ctrl.Rule(self.error['NS'] & self.delta_error['PM'], self.output['ZE']),
            ctrl.Rule(self.error['NS'] & self.delta_error['PB'], self.output['ZE']),

            ctrl.Rule(self.error['ZE'] & self.delta_error['NB'], self.output['NB']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['NM'], self.output['NM']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['NS'], self.output['NM']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['ZE'], self.output['ZE']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['PS'], self.output['ZE']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['PM'], self.output['PS']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['PB'], self.output['PS']),

            ctrl.Rule(self.error['PS'] & self.delta_error['NB'], self.output['NM']),
            ctrl.Rule(self.error['PS'] & self.delta_error['NM'], self.output['NM']),
            ctrl.Rule(self.error['PS'] & self.delta_error['NS'], self.output['NS']),
            ctrl.Rule(self.error['PS'] & self.delta_error['ZE'], self.output['ZE']),
            ctrl.Rule(self.error['PS'] & self.delta_error['PS'], self.output['PS']),
            ctrl.Rule(self.error['PS'] & self.delta_error['PM'], self.output['PS']),
            ctrl.Rule(self.error['PS'] & self.delta_error['PB'], self.output['PM']),

            ctrl.Rule(self.error['PM'] & self.delta_error['NB'], self.output['NS']),
            ctrl.Rule(self.error['PM'] & self.delta_error['NM'], self.output['NS']),
            ctrl.Rule(self.error['PM'] & self.delta_error['NS'], self.output['ZE']),
            ctrl.Rule(self.error['PM'] & self.delta_error['ZE'], self.output['PS']),
            ctrl.Rule(self.error['PM'] & self.delta_error['PS'], self.output['PS']),
            ctrl.Rule(self.error['PM'] & self.delta_error['PM'], self.output['PM']),
            ctrl.Rule(self.error['PM'] & self.delta_error['PB'], self.output['PB']),

            ctrl.Rule(self.error['PB'] & self.delta_error['NB'], self.output['ZE']),
            ctrl.Rule(self.error['PB'] & self.delta_error['NM'], self.output['ZE']),
            ctrl.Rule(self.error['PB'] & self.delta_error['NS'], self.output['PS']),
            ctrl.Rule(self.error['PB'] & self.delta_error['ZE'], self.output['PS']),
            ctrl.Rule(self.error['PB'] & self.delta_error['PS'], self.output['PM']),
            ctrl.Rule(self.error['PB'] & self.delta_error['PM'], self.output['PB']),
            ctrl.Rule(self.error['PB'] & self.delta_error['PB'], self.output['PB']),
        ]

        # Creación del sistema de control difuso
        self.control_system = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.control_system)

    def update(self, vc):
        error = self.setpoint - vc
        delta_error = error - self.last_error

        self.simulation.input['error'] = error
        self.simulation.input['delta_error'] = delta_error
        self.simulation.compute()

        self.last_error = error
        return (self.setpoint, self.simulation.output['output'])

class AnimationPlot:
    def __init__(self):
        self.sp_data = []
        self.y_data = []
        self.t_data = []
        self.signal = 0.0
        self.LIMITE_MAX = 4000
        self.LIMITE_MIN = 475
        self.setpoint = 0.0
        self.time = 0.1
        self.dt = 0.05

    def animate(self, _, ard, pid):
        arduinoData_string = ard.readline().decode('ascii')
        try:
            arduinoData_float = float(arduinoData_string)
            self.setpoint, self.signal = pid.update(arduinoData_float)
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

    def rpmToAnalog(self, RPM):
        return int((RPM / self.LIMITE_MAX) * 255)

    def getPlotFormat(self):
        ax.set_ylim([0, (self.LIMITE_MAX + 250)])
        ax.set_title("Gráfica PID para controlar RPM")

fig = plt.figure('Control PID Difuso')
ax = fig.add_subplot(111)

realTimePlot = AnimationPlot()
arduino = serial.Serial("COM7", 115200)
sleep(2)
arduino.write(b'55')
system('cls')

# Inicializar el controlador PID difuso
pid_fuzzy = FuzzyPID(Setpoint=1400)

# Correr la animación con el PID difuso
ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=100, fargs=(arduino, pid_fuzzy), interval=10)
plt.draw()
plt.show()

arduino.write(b'0')
sleep(0.1)
arduino.write(b'0')
arduino.close()
system('cls')
