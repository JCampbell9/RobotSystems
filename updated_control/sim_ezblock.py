import logging

#
# class _Basic_class(object):
#     _class_name = '_Basic_class'
#     DEBUG_LEVELS = {'debug': logging.DEBUG,
#               'info': logging.INFO,
#               'warning': logging.WARNING,
#               'error': logging.ERROR,
#               'critical': logging.CRITICAL,
#               }
#     DEBUG_NAMES = ['critical', 'error', 'warning', 'info', 'debug']
#
#     def __init__(self):
#         self._debug_level = 0
#         self.logger = logging.getLogger(self._class_name)
#         self.ch = logging.StreamHandler()
#         form = "%(asctime)s	[%(levelname)s]	%(message)s"
#         self.formatter = logging.Formatter(form)
#         self.ch.setFormatter(self.formatter)
#         self.logger.addHandler(self.ch)
#         self._debug    = self.logger.debug
#         self._info     = self.logger.info
#         self._warning  = self.logger.warning
#         self._error    = self.logger.error
#         self._critical = self.logger.critical
#
#     @property
#     def debug(self):
#         return self._debug_level
#
#     @debug.setter
#     def debug(self, debug):
#         if debug in range(5):
#             self._debug_level = self.DEBUG_NAMES[debug]
#         elif debug in self.DEBUG_NAMES:
#             self._debug_level = debug
#         else:
#             raise ValueError('Debug value must be 0(critical), 1(error), 2(warning), 3(info) or 4(debug), not \"{0}\".'.format(debug))
#         self.logger.setLevel(self.DEBUG_LEVELS[self._debug_level])
#         self.ch.setLevel(self.DEBUG_LEVELS[self._debug_level])
#         self._debug('Set logging level to [%s]' % self._debug_level)
#
#     def run_command(self, cmd):
#         import subprocess
#         p = subprocess.Popen(
#             cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
#         result = p.stdout.read().decode('utf-8')
#         status = p.poll()
#         # print(result)
#         # print(status)
#         return status, result
#
#     def map(self, x, in_min, in_max, out_min, out_max):
#         return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
#

class Servo():

    def __init__(self, pwm):
        # super().__init__()
        self.pwm = pwm

    def angle(self, angle):
        if not (isinstance(angle, int) or isinstance(angle, float)):
            raise ValueError("Angle value should be int or float value, not %s" % type(angle))
        if angle < -90:
            angle = -90
        if angle > 90:
            angle = 90

class PWM:
    ADDR = 0x14

    def __init__(self, channel, debug="critical"):
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
            else:
                raise ValueError("PWM channel should be between [P1, P14], not {0}".format(channel))
        self.debug = debug
        # self._debug("PWM address: {:02X}".format(self.ADDR))
        self.channel = channel
        self.timer = int(channel / 4)
        self._pulse_width = 0
        self._freq = 50
        self.freq(50)

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        if len(freq) == 0:
            return self._freq
        else:
            pass

    def prescaler(self, *prescaler):
        if len(prescaler) == 0:
            return self._prescaler
        else:
            pass

    def period(self, *arr):
        global timer
        if len(arr) == 0:
            return timer[self.timer]["arr"]
        else:
            pass

    def pulse_width(self, *pulse_width):
        if len(pulse_width) == 0:
            return self._pulse_width
        else:
            pass

    def pulse_width_percent(self, *pulse_width_percent):
        global timer
        if len(pulse_width_percent) == 0:
            return self._pulse_width_percent
        else:
            pass

class Pin:

    def __init__(self, *value):
        pass

    def check_board_type(self):
        pass

    def init(self, modem, pull):
        pass

    def dict(selfself, * dict):
        pass

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        if len(value) == 0:
            return True
        else:
            return value[0]

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        pass

    def pull(self, *value):
        return 1

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        return "GPIO is some pin number"

    def names(self):
        return ["some name", "name of board"]

class ADC:

    def __init__(self, chn):
        pass

    def read(self):
        return 3  #random number

    def read_voltage(self):
        return self.read()*3.3/4095

