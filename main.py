from time import sleep
from os import system
import serial, numpy as np, matplotlib.pyplot as plt, matplotlib.animation as animation

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
        #dt Derivativo
        derivative = (self.error - self.last_error) / self.dt

        if(abs(self.output) >= LIMITE_MAX and (((self.error>=0.0) and (self.integral>=0.0))or((self.error<0.0) and (self.integral<0.0)))):
            if(self.antiWindup):
                #Sin integración dt Integrativo
                self.integral = self.integral
            else:
                #Si antiWindup == False then integración rectangular (dt Integrativo)
                self.integral += self.error * self.dt
        else:
            #Integración rectangular (dt Integrativo)
            self.integral += self.error * self.dt
        #    u(n)   |  u(n-1)     |    parte proporcional  |    parte integrativa    |   parte derivativa 
        self.output = self.output + ( self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative) * self.dt
        self.last_error = self.error
        if self.output >= LIMITE_MAX:
            self.output = LIMITE_MAX
        if self.output <= LIMITE_MIN:
            self.output = LIMITE_MIN
        return (self.setpoint, self.output)

class PIDTest:
    def __init__(self, Kp: float, Ti: float, Td: float, Setpoint: int, antiWindup: bool = False):
        self.Kp: float = Kp
        self.Ti: float = Ti
        self.Td: float = Td
        self.setpoint: int = Setpoint
        self.dt: float = 0.1
        self.last_error: float = 0.0
        self.integral: float = 0.0
        self.antiWindup: bool = antiWindup
        self.error: float = 0.0
        self.output: float = 0.0

    def update(self, vc, LIMITE_MAX: float, LIMITE_MIN: float):
        self.error = self.setpoint - vc
        #dt Derivativo
        derivative = (self.error - self.last_error) / self.dt

        if(abs(self.output) >= LIMITE_MAX and (((self.error>=0.0) and (self.integral>=0.0))or((self.error<0.0) and (self.integral<0.0)))):
            if(self.antiWindup):
                #Sin integración dt Integrativo
                self.integral = self.integral
            else:
                #Si antiWindup == False then integración rectangular (dt Integrativo)
                self.integral += self.error * self.dt
        else:
            #Integración rectangular (dt Integrativo)
            self.integral += self.error * self.dt
        #    u(n)   |  u(n-1)     |    parte proporcional  |    parte integrativa    |   parte derivativa 
        self.output = self.output + ( self.Kp * self.error + (self.Kp/ self.Ti)* self.integral + (self.Kp * self.Td) * derivative) * self.dt
        self.last_error = self.error
        if self.output >= LIMITE_MAX:
            self.output = LIMITE_MAX
        if self.output <= LIMITE_MIN:
            self.output = LIMITE_MIN
        return (self.setpoint, self.output)

class AnimationPlot:
    def __init__(self):                             #Declaración de variables iniciales
        self.sp_data: list[float] = []
        #self.sp_data: list[float] = np.array([])
        self.y_data: list[float] = []
        self.t_data: list[float] = []
        self.signal: float = 0.0
        self.LIMITE_MAX: float = 4000
        #self.LIMITE_MAX: float = 163.0
        #self.LIMITE_MIN: float = 157.5
        #self.LIMITE_MIN: float = 1350
        self.LIMITE_MIN: float = 475
        #self.LIMITE_MIN: float = 51.0
        self.setpoint: float = 0.0
        self.time: float = 0.1
        self.dt: float = 0.05
        

    def animate(self, _, ard, pid): #Por referencia FuncAnimation incluye un argumento que hay que omitir, es un contador
        arduinoData_string = ard.readline().decode('ascii') # Decode receive Arduino data as a formatted string

        try:
            arduinoData_float = float(arduinoData_string)   # Convert to float
            self.setpoint, self.signal = pid.update(arduinoData_float, self.LIMITE_MAX, self.LIMITE_MIN)
            self.y_data.append(arduinoData_float)           # Add to the list holding the fixed number of points to animate
            #np.append(self.y_data,arduinoData_float)
            self.sp_data.append(self.setpoint)
            #np.append(self.sp_data, self.setpoint)
            self.t_data.append(self.time)
            if len(self.t_data) > 1000:         #mostramos últimos 1000 sensados
                self.t_data = self.t_data[1:]
                self.y_data = self.y_data[1:]
                self.sp_data = self.sp_data[1:]
            self.time += self.dt
            x = self.rpmToAnalog(int(self.signal))
            if x >= 255:
                x=255
            elif x <= 55:
                x = 55
            ard.write(bytes(str(x), 'utf-8'))
        
            ax.clear()                          # Clear last data frame
            self.getPlotFormat()
            ax.plot(self.t_data, self.y_data, c='b', label=f"Lectura (RPM): {int(arduinoData_float)}")   # Plot new data frame
            ax.plot(self.t_data, self.sp_data  , c='r', label=f"Setpoint: {int(self.setpoint)}")
            ax.legend(loc='upper right')
        except:                                 # Pass if data point is bad                               
            pass
    
    def rpmToAnalog(self, RPM: int):            #Convert RPM to Analog, send to Arduino 🥵
        return int((RPM/self.LIMITE_MAX)*255)

    def getPlotFormat(self):
        ax.set_ylim([0, (self.LIMITE_MAX + 250)])       # Set Y axis limit of plot
        #ax.set_ylim([0, (self.LIMITE_MAX + 14)])       # Set Y axis limit of plot
        ax.set_title("Gráfica PID para controlar RPM")
                                                        
fig = plt.figure('Control PID Kp=1.0, Ki=1.13, Kd=0.5, Setpoint=2510, antiWindup=True')                 # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                       # Add subplot to main fig window

realTimePlot = AnimationPlot()

arduino = serial.Serial("COM7", 115200)         # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
sleep(2)                                        # Time delay for Arduino Serial initialization
arduino.write(b'50')
system('cls')
#pidInit = PID(Kp=2, Ki=0.12, Kd=0.01, Setpoint=1500)
#pidInit = PID(Kp=2.0, Ki=1.1, Kd=0.1, Setpoint=1600, antiWindup=True)
#pidInit = PID(Kp=2.0, Ki=1.6, Kd=0.01, Setpoint=1517)
#pidInit = PID(Kp=2.0, Ki=1.6, Kd=0.01, Setpoint=512)
#pidInit = PID(Kp=2.0, Ki=1.15, Kd=2.5, Setpoint=355.5)
#pidInit = PID(Kp=1.0, Ki=1.15, Kd=0.5, Setpoint=1350)
#pidInit = PID(Kp=1.0, Ki=1.15, Kd=0.5, Setpoint=2100)
#pidInit = PID(Kp=1.0, Ki=1.13, Kd=0.5, Setpoint=1500)
#pidInit = PID(Kp=1.0, Ki=1.13, Kd=0.5, Setpoint=1500, antiWindup=True)
#pidInit = PID(Kp=1.0, Ki=1.13, Kd=0.5, Setpoint=3120)
pidInit = PID(Kp=1.0, Ki=1.13, Kd=0.5, Setpoint=2510, antiWindup=True)
#pidInit = PID(Kp=2, Ki=0.12, Kd=0.01, Setpoint=1500, antiWindup=True)
#pidInit = PID(Kp=2, Ki=0.12, Kd=0.01, Setpoint=600)
#pidInit = PID(Kp=2, Ki=0.12, Kd=0.01, Setpoint=600, antiWindup=True)

ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=100, fargs=(arduino, pidInit), interval=10)
                            # Matplotlib Animation Fuction that takes takes care of real time plot.
                            # Note that 'fargs' parameter is where we pass in our dataList and Serial object.
plt.draw()
plt.show()                  # Keep Matplotlib plot persistent on screen until it is closed
arduino.write(b'0')
sleep(0.1)
arduino.write(b'0')
arduino.close()             # Close Serial connection when plot is closed
system('cls')
