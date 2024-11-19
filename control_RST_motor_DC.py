from machine import Pin, I2C, Timer, UART, PWM
import machine
import utime
import math as m


uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))
uart.init(9600, bits=8, parity=None, stop=1)

class Motor():
    
    max_duty_u16=65536
    
    def __init__(self, pin_num_pwm, pin_num_in1, pin_num_in2,sensorA_pin, sensorB_pin, freq=1000):

        # def. attributes 
        self._pwm = PWM(Pin(pin_num_pwm))
        self._pin_in1 = Pin(pin_num_in1, Pin.OUT)
        self._pin_in2 = Pin(pin_num_in2, Pin.OUT)
        self._pwm.freq(freq)
        self.sensorA = Pin(sensorA_pin, Pin.IN)
        self.sensorB = Pin(sensorB_pin, Pin.IN)

        # Encoder and motor parameters
        self.ppr = 16 # 16 picos por revolucion
        self.decoder_number = 1 # number of edge count (falling, rising, both)*sensor 
        self.gear_ratio = 18.75

        # ticks counter
        self.encoder_count = 0
        # speed time variables
        self.prevT = 0
        self.posPrev = 0
          # IRQ initialization
        self.sensorA.irq(trigger=Pin.IRQ_FALLING,handler=self._motorEncoderCallbackL)
        
        
    def set_motor(self, direccion, speed):    
        #  functions to set motor controller signals
        #print("coucou")
        if direccion == 1:
            self._pin_in1.off()
            self._pin_in2.on()
            
        elif direccion == -1:
            self._pin_in1.on()
            self._pin_in2.off()
        else:
            self._pin_in1.off()
            self._pin_in2.off()

        self._pwm.duty_u16(self.speedToU16(abs(speed)))
    
    @classmethod
    # Motor().speedToU16
    # mymotor=Motor(parametros)
    # mymotor.set_motor
    
    def speedToU16(cls, speed):
        # Return a duty_u16 value based on percentage speed
#         print(int(speed))
        return int(speed * cls.max_duty_u16/100)

    def _motorEncoderCallbackL(self,pin):
        if self.sensorB.value() > 0:
            self.encoder_count += 1
        else:
            self.encoder_count -= 1


    def encoder2speed(self):

        currT = utime.ticks_ms()
        deltaT = utime.ticks_diff(currT, self.prevT)*0.001 # en s

        self.prevT = currT # current time

        #pos_filt = lp_ticks.filt(pos)

        vel = ((self.encoder_count - self.posPrev)/deltaT)

       # storage values
        self.posPrev = self.encoder_count #; // current tick 
        # vel in rpm
        self.vel_porcentaje = ((vel)/(self.ppr * self.decoder_number * self.gear_ratio))*(60)*100/540 #;
        #print(self.vel_porcentaje)
        return self.vel_porcentaje
    
    
    def encoder2pos(self): # Nos da la posicion en radianes
        
        self.pos = (self.encoder_count)/(self.ppr * self.decoder_number * self.gear_ratio)*2*m.pi
        return self.pos


def main():
     
    control=input("que control elegir: \n1=RST posicional\n2=RSTincremental\n3=enmascara de RST en PID\n4=PID\n5=control de posicion por RST posicional\n6=para el motor\n")

     # Built-in LED initialization
    led = Pin(25, Pin.OUT)

    # motor pinout definition
    pin_pwm,  pin_in1, pin_in2, sensorA_pin, sensorB_pin, freq = 12, 13, 14, 3, 2, 1000

    # motor object
    motor_l = Motor(pin_pwm,  pin_in1, pin_in2, sensorA_pin, sensorB_pin, freq)

    # time setup
    previous_time = 0
    sample_time = 30


    # speed variable
    potentiometer = machine.ADC(28)
    # scale input
    conversion_factor = 100 / (65535)
    
    
    
    # Inicializar variables controlador
    uk1 = 0
    error1 = 0
    error2 = 0
    yk1=0
    ypk1=0
    
    
    #parametros del controlador PID:
    q0 = 0.07
    q1 = 0.14
    q2 = 0.084
    
    #inicializamos listas para gaficar la respuesta
    t=0

    while True:
        
        led.off()
        # time management
        current_time = utime.ticks_ms()
        delta_time = utime.ticks_diff(current_time, previous_time)

        # sampling conditional
        if delta_time > sample_time:
            
            
            # speed measurement
            #print(motor_l.encoder2speed())
            led.on()

            # potentiometer scale 
            set_speed = map(potentiometer.read_u16() * conversion_factor,0,100,-100,100)

            vel=motor_l.encoder2speed()
            yk=vel
            xk=set_speed


            set_pos=map(potentiometer.read_u16() * conversion_factor,0,100,-2*m.pi,2*m.pi)
            
            xpk= set_pos  # setpoint en radianes (entre -5.4 y 5.4 en este caso)
            ypk=motor_l.encoder2pos() # valor en radianes
            
            #modelo del motor: G=1.19/(0.335*s+1)
            
            
            if control=="1": # RST posicional
                uk=0.57*xk+0.27*yk
                #uk=3.57*xk-1.77*yk
                
            elif control=="2":  #RST incremental
                uk=0.57*xk-1.148*yk+0.58*yk1 + uk1
                #uk=12.259*xk-7.039*yk-5.22*yk1 + uk1
                yk1=yk
                uk1=uk

            elif control=="3": # enmascara de un RST en un PID
                error=setP*0.57-1.148*yk+0.58*yk1
                uk = 1.148*error-0.58*error1+uk1
                error1 = error
                uk1 = uk
                yk1=yk
            
              
            elif control=="4": #control PID
                error = setP - vel
                uk = q0*error+q1*error1+q2*error2+uk1
                error2 = error1
                error1 = error
                uk1 = uk
                
                
            elif control=="5": # control de posicion por RST posicional
                ek=23.7*xpk-41.8*ypk+18.2*ypk1
                uk=-0.4*uk1+ek
                uk1=uk
                ypk1=ypk
                #print(ypk1)
                
            
            elif control=="6": # paramos el motor
                uk=0
                
                
            error_vel = vel-set_speed
            error_pos= ypk-set_pos
            
            #print("error: ",error, "setpoint: ",setP)
            #print("setpoint_vel",setP,'velocidad',yk,"error_vel",error_vel)
            
            
            if uk > 100:
                uk = 100
            
            elif uk < -100:
                uk = -100
            
            if uk < 0: 
                motor_l.set_motor(-1,abs(uk))
                #print('reverse')
            else:
                motor_l.set_motor(1,abs(uk))


            print("setpoint_posicion",xpk,'posicion',ypk,"error_pos",error_pos)


            previous_time = current_time
    
    print('\n------- End Program --------\n')
 

def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    
if __name__ == '__main__':
    main()