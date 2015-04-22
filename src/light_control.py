import math

class Lights():

    def __init__(self, ser):
        self.ser = ser
        self.start_char = 254
        self.stop_char = 255
        self.colors = {"red": [255,0,0],
                       "green": [0,255,0],
                       "blue": [0,0,255],
                       "amber": [255,70,0],
                       "turquoise": [0,255,150],
                       "purple": [255,0,255],
                       "white": [255,255,255],
                       "orange": [255,30,0]}

    def solid_color(self,light, rgb, brightness = 1):
        '''turn a given light the given rgb value'''
        red, green, blue = self.get_rgb(rgb, brightness)
        self.send_msg(light, red, green, blue, 0)

    def blink(self,light, rgb, prd, brightness = 1):
        red, green, blue = self.get_rgb(rgb, brightness)
        p_code = self.encode_prd(prd)
        self.send_msg(light, red, green, blue, p_code)

    def fade(self,light, rgb, prd, brightness = 1):
        red, green, blue = self.get_rgb(rgb, brightness)
        p_code = self.encode_prd(prd*2)+128
        self.send_msg(light, red, green, blue, p_code)

    def get_rgb(self, rgb, brightness):
        brightness = min(1, max(0, brightness))
        if type(rgb) == str:
            return [brightness*color for color in self.colors[rgb]]
        else:
            return [brightness*color for color in rgb]

    def encode_prd(self,prd):
        prd = max(1000*prd, 2) # get the period in ms, at least 2 ms
        p_code = int(math.log(prd,2))-1;
        return min(127,max(1,p_code))

    def send_msg(self,light,red,green,blue,p_code):
        msg = [self.start_char,
               light,
               red,
               green,
               blue,
               p_code,
               self.stop_char]

        self.ser.write(''.join(chr(b) for b in msg))
