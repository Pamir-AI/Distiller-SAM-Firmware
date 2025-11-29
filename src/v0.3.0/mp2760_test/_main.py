import machine
import utime
import neopixel

CM_en = machine.Pin(3, machine.Pin.OUT, value=0)

debug_rgb = neopixel.NeoPixel(machine.Pin(20), 1)
i2c = machine.I2C(0, sda=machine.Pin(24), scl=machine.Pin(25))
i2c.writeto_mem(0x5c, 0x08, bytes([0xff,0xff]))

debug_rgb[0] = (255,0,0)
debug_rgb.write()
utime.sleep_ms(200)


# CM_en.on()
CM_en = machine.Pin(3, machine.Pin.IN, machine.Pin.PULL_UP)

while True:
    
    
    debug_rgb[0] = (255,255,0)
    debug_rgb.write()
    utime.sleep_ms(200)
    
    debug_rgb[0] = (255,0,255)
    debug_rgb.write()
    utime.sleep_ms(200)
