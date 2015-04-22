import math

class Lights():
  '''Talks to an arduino to control the lights'''

    def __init__(self, ser, num_lights=8):
        self.ser = ser # serial.Serial interface to the arduino
        self.start_char = 254 # char arduino recognizes as valid command start
        self.stop_char = 255 # char arduino recognizes as valid command end
        self.num_lights = num_lights

        # these colors can be referenced by name, rather than rgb value
        self.colors = {"red": [255,0,0],
                       "green": [0,255,0],
                       "blue": [0,0,255],
                       "amber": [255,70,0],
                       "turquoise": [0,255,150],
                       "purple": [255,0,255],
                       "white": [255,255,255],
                       "orange": [255,30,0]}

    def solid_color(self, light, rgb, brightness = 1):
        '''turn a given light the given rgb value, optionally scaled by the 
        brightness. rgb can be either a name from Lights.colors or 
        [red,green,blue] integer values between 0 and 255'''
        if light >= 0 and light < num_lights:
            red, green, blue = self.get_rgb(rgb, brightness)
            self.send_msg(light, red, green, blue, 0)

    def blink(self,light, rgb, prd, brightness = 1):
        '''blink a given light the given rgb value, with a period in seconds 
        specified by prd, optionally scaled by the brightness. rgb can be either 
        a name from Lights.colors or [red,green,blue] integer values between 0 
        and 255'''
        if light >= 0 and light < num_lights:
            red, green, blue = self.get_rgb(rgb, brightness)
            p_code = self.encode_prd(prd)
            self.send_msg(light, red, green, blue, p_code)

    def fade(self,light, rgb, prd, brightness = 1):
        '''fade a given light in and out with the given rgb value, with a period 
        in seconds specified by prd. The maximum rgb value can be optionally 
        scaled by the brightness. rgb can be either a name from Lights.colors or 
        [red,green,blue] integer values between 0 and 255'''
        if light >= 0 and light < num_lights:
            red, green, blue = self.get_rgb(rgb, brightness)
            p_code = min(self.encode_prd(prd*2)+128, 255)
            self.send_msg(light, red, green, blue, p_code)

    def get_rgb(self, rgb, brightness):
        '''takes either a color name or [red, green, blue], and returns rgb 
        values scaled by brightness'''
        brightness = min(1, max(0, brightness))
        if type(rgb) == str:
            return [brightness*color for color in self.colors[rgb]]
        else:
            return [brightness*color for color in rgb]

    def encode_prd(self,prd):
        '''takes the desired period in seconds, and encodes it for transmission 
        to the arduino. (The arduino only operates in periods which are powers 
        of 2 in milliseconds, and the period is encoded by giving it the power 
        of 2 to use)'''
        prd = max(1000*prd, 2) # get the period in ms, at least 2 ms
        p_code = int(math.log(prd,2))-1;
        return min(127,max(1,p_code))

    def send_msg(self,light,red,green,blue,p_code):
        '''construct and send the message to the arduino'''
        msg = [self.start_char,
               light,
               red,
               green,
               blue,
               p_code,
               self.stop_char]

        self.ser.write(''.join(chr(b) for b in msg))
