from picarx_controls import *
from rossros import *


if __name__ == '__main__':

    sensor_function = SensorControl()
    sensor_interp = Interpreter()
    controller = Controller()

    controller.motor_controller.forward(40)

    sonic_value = UltrasonicSensor
    sonic_interp = UltrasonicInterpreter(.5)

    runtimer_bus = Bus(False, 'run_timer')

    pixel_sensor_bus = Bus([0, 0, 0], '3_pixel_values')
    pixel_interp_bus = Bus(0, '3_pixel_interp')
    sonic_value_bus = Bus(10, 'ultrasonic distance')
    sonic_interp_bus = Bus(False, 'is there an obstacle')

    timer = Timer(runtimer_bus, 5, 0, runtimer_bus, 'timer')

    Esensor_values = Producer(sensor_function, pixel_sensor_bus, 0, runtimer_bus, 'gets the 3pixel sensor value')
    Esensor_interp = ConsumerProducer(sensor_interp, pixel_sensor_bus, pixel_interp_bus, 0, runtimer_bus, 'pixel_interp')
    Esonic_value = Producer(sonic_value, sonic_value_bus, 0, runtimer_bus, 'ultrasonic value')
    Esonic_interp = ConsumerProducer(sonic_interp, sonic_value_bus, sonic_interp_bus, 0, runtimer_bus, 'ultrasonic interp')
    Econtroller = Consumer(controller, [pixel_interp_bus, sonic_interp_bus], 0, runtimer_bus, "controls steering of the car")

    runConcurrently([Esensor_values, Esensor_interp, Econtroller, Esonic_value, Esonic_interp, timer])
