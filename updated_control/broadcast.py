from picarx_controls import *
import time
import concurrent.futures
from threading import Lock


class Broadcast:
    """
    class for doing multitasking, and allow for communication between them
    """

    def __init__(self):
        self.adc_message = []
        self.turn_message = 0

    def write(self, which_message, updated_message):
        """
        This method updates the message to the new message
        :param which_message: ADC_values, or turn_value
        :param updated_message: new message
        :return:
        """
        if which_message == "ADC_values":
            self.adc_message = updated_message
        elif which_message == "turn_value":
            self.turn_message = updated_message
        # self.message.append(updated_message)

    def read(self, which_message):
        """
        Allows for reading the message
        :param which_message: ADC_values, or turn_value
        :return:
        """
        if which_message == "ADC_values":
            return self.adc_message
        elif which_message == "turn_value":
            return self.turn_message


def sensor_function(broadcast, time_delay):
    """
    A loop that gets the sensor values and writes them to the broadcast instance
    :param broadcast: Broadcast instance
    :param time_delay: what is the loop delay
    :return: none
    """

    sensor = SensorControl
    lock = Lock()
    while True:
        with lock:
            values = sensor.get_adc_values
        broadcast.write("ADC_values", values)

        time.sleep(time_delay)


def interpreter_function(broadcast, time_delay):
    """
    Reads the sensor values and writes the updated turn value
    :param broadcast: Broadcast instance
    :param time_delay: what is the loop delay
    :return: none
    """
    interp = Interpreter

    while True:
        broadcast.write("turn_value", interp.robot_relative_to_line(broadcast.read("ADC_values")))

        time.sleep(time_delay)


def control_function(broadcast, time_delay):
    """
    Reads the turn value and updates the turning of the car
    :param broadcast: Broadcast instance
    :param time_delay: what is the loop delay
    :return: none
    """
    control = Controller()

    while True:
        offset = broadcast.read("turn_value")
        control.line_follower(offset=offset)
        time.sleep(time_delay)


if __name__ == "__main__":

    cast = Broadcast

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor_function, cast, .1)
        eInterpreter = executor.submit(interpreter_function, cast, .1)
        eControl = executor.submit(control_function, cast, .1)
