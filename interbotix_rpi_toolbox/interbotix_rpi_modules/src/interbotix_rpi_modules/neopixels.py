import rospy
from interbotix_rpi_modules.msg import PixelCommands

class InterbotixRpiPixelInterface(object):
    def __init__(self, robot_name):
        self.pub_pixels = rospy.Publisher("/" + robot_name + "/commands/pixels", PixelCommands, queue_size=5)
        rospy.sleep(1)

    def set_color(self, pixel=0, color=0x000000, set_all_leds=False):
        msg = PixelCommands()
        msg.cmd_type = 'color'
        msg.set_all_leds = set_all_leds
        msg.pixel = pixel
        msg.color = color
        self.pub_pixels.publish(msg)

    def set_brightness(self, brightness=0):
        msg = PixelCommands()
        msg.cmd_type = 'brightness'
        msg.brightness = brightness
        self.pub_pixels.publish(msg)

    def pulse(self, iterations=5, period=10):
        msg = PixelCommands()
        msg.cmd_type = 'pulse'
        msg.period = period
        msg.iterations = iterations
        self.pub_pixels.publish(msg)

    def blink(self, pixel=0, set_all_leds=False, period=500, iterations=3):
        msg = PixelCommands()
        msg.cmd_type = 'blink'
        msg.set_all_leds = set_all_leds
        msg.pixel = pixel
        msg.period = period
        msg.iterations = iterations
        self.pub_pixels.publish(msg)
