import machine
import utime
#TODO unblock the eink await 

class einkDSP_SAM:

    def __init__(self) -> None:
        self.oldData = 0x00

        # Pin Definition
        self.DC_PIN = machine.Pin(12, machine.Pin.OUT)
        self.RST_PIN = machine.Pin(11, machine.Pin.OUT)
        self.BUSY_PIN = machine.Pin(10, machine.Pin.IN, machine.Pin.PULL_UP)

        # Updated dimensions to match Display_EPD_W21
        self.EPD_WIDTH = 128
        self.EPD_HEIGHT = 250
        self.EPD_ARRAY = self.EPD_WIDTH * self.EPD_HEIGHT // 8
    
        # Initialize SPI
        self.spi = machine.SPI(1,baudrate=25000000,sck=machine.Pin(14, machine.Pin.OUT), mosi=machine.Pin(15, machine.Pin.OUT), miso=machine.Pin(8, machine.Pin.OUT))
        self.cs = machine.Pin(13, mode=machine.Pin.OUT, value=1)
        self.init = True
        self.watchdogCounter = 0
        
        # Remove old LUT tables as Display_EPD_W21 doesn't use them
    
    def de_init(self):
        self.spi.deinit()
        self.DC_PIN = machine.Pin(12, machine.Pin.IN, None)
        self.RST_PIN = machine.Pin(11, machine.Pin.IN, None)
        self.BUSY_PIN = machine.Pin(10, machine.Pin.IN, None)
        self.cs = machine.Pin(13, machine.Pin.IN, None)
        machine.Pin(14, machine.Pin.IN, None)
        machine.Pin(15, machine.Pin.IN, None)
        machine.Pin(8, machine.Pin.IN, None)
        self.init = False
            
    def re_init(self):
        self.DC_PIN = machine.Pin(12, machine.Pin.OUT)
        self.RST_PIN = machine.Pin(11, machine.Pin.OUT)
        self.BUSY_PIN = machine.Pin(10, machine.Pin.IN, machine.Pin.PULL_UP)

        self.spi = machine.SPI(1,baudrate=25000000,sck=machine.Pin(14, machine.Pin.OUT), mosi=machine.Pin(15, machine.Pin.OUT), miso=machine.Pin(8, machine.Pin.OUT))
        self.cs = machine.Pin(13, mode=machine.Pin.OUT, value=1) 
        self.init = True
        
    def SPI_Delay(self):
        utime.sleep_us(10)  # 10 microseconds

    def SPI_Write(self, value):
        self.cs.low()
        self.spi.write(bytearray([value]))
        self.cs.high()

    def epd_w21_write_cmd(self, command):
        self.SPI_Delay()
        self.DC_PIN.low()
        self.SPI_Write(command)

    def epd_w21_write_data(self, data):
        self.SPI_Delay()
        self.DC_PIN.high()
        self.SPI_Write(data)

    def delay_xms(self, xms):
        utime.sleep_us(xms*1000)

    def lcd_chkstatus(self):
        # Busy function - adapted from Epaper_READBUSY()
        while self.BUSY_PIN.value() == 1 and self.watchdogCounter < 1000:  # =1 BUSY
            self.delay_xms(10)
            self.watchdogCounter += 1
        print(f"counter: {self.watchdogCounter}")
        self.watchdogCounter = 0

    # Full screen refresh initialization - adapted from EPD_HW_Init()
    def epd_init(self):
        # Module reset
        self.RST_PIN.low()
        self.delay_xms(10)  # At least 10ms delay
        self.RST_PIN.high()
        self.delay_xms(10)  # At least 10ms delay
        
        self.lcd_chkstatus()
        self.epd_w21_write_cmd(0x12)  # SWRESET
        self.lcd_chkstatus()
        
        self.epd_w21_write_cmd(0x01)  # Driver output control
        self.epd_w21_write_data((self.EPD_HEIGHT-1) % 256)
        self.epd_w21_write_data((self.EPD_HEIGHT-1) // 256)
        self.epd_w21_write_data(0x00)

        self.epd_w21_write_cmd(0x11)  # data entry mode
        self.epd_w21_write_data(0x01)  # Normal mode

        self.epd_w21_write_cmd(0x44)  # set Ram-X address start/end position
        self.epd_w21_write_data(0x00)                 # Start first
        self.epd_w21_write_data(self.EPD_WIDTH//8-1)  # End second

        self.epd_w21_write_cmd(0x45)  # set Ram-Y address start/end position
        self.epd_w21_write_data((self.EPD_HEIGHT-1) % 256)  # Start with height-1
        self.epd_w21_write_data((self.EPD_HEIGHT-1) // 256)
        self.epd_w21_write_data(0x00)                       # End with 0
        self.epd_w21_write_data(0x00)

        self.epd_w21_write_cmd(0x3C)  # BorderWavefrom
        self.epd_w21_write_data(0x05)

        self.epd_w21_write_cmd(0x21)  # Display update control
        self.epd_w21_write_data(0x00)
        self.epd_w21_write_data(0x80)  # Normal mode

        self.epd_w21_write_cmd(0x18)  # Read built-in temperature sensor
        self.epd_w21_write_data(0x80)

        self.epd_w21_write_cmd(0x4E)  # set RAM x address count
        self.epd_w21_write_data(0x00)  # Start at 0
            
        self.epd_w21_write_cmd(0x4F)  # set RAM y address count
        self.epd_w21_write_data((self.EPD_HEIGHT-1) % 256)  # Start at height-1
        self.epd_w21_write_data((self.EPD_HEIGHT-1) // 256)
        self.lcd_chkstatus()

    # Fast refresh initialization - adapted from EPD_HW_Init_Fast2() (renamed from Fast2 as it's faster)
    def epd_init_fast(self):
        # Module reset
        self.RST_PIN.low()
        self.delay_xms(10)  # At least 10ms delay
        self.RST_PIN.high()
        self.delay_xms(10)  # At least 10ms delay
        
        self.epd_w21_write_cmd(0x12)  # SWRESET
        self.lcd_chkstatus()
        
        self.epd_w21_write_cmd(0x18)  # Read built-in temperature sensor
        self.epd_w21_write_data(0x80)
        
        self.epd_w21_write_cmd(0x22)  # Load temperature value
        self.epd_w21_write_data(0xB1)
        self.epd_w21_write_cmd(0x20)
        self.lcd_chkstatus()

        self.epd_w21_write_cmd(0x1A)  # Write to temperature register
        self.epd_w21_write_data(0x5A)  # Fast2 value (0x5A vs 0x64 in original fast)
        self.epd_w21_write_data(0x00)
        
        self.epd_w21_write_cmd(0x22)  # Load temperature value
        self.epd_w21_write_data(0x91)
        self.epd_w21_write_cmd(0x20)
        self.lcd_chkstatus()



    # Update functions - adapted from Arduino version
    def epd_update(self):
        # Full screen refresh update function
        self.epd_w21_write_cmd(0x22)  # Display Update Control
        self.epd_w21_write_data(0xF7)
        self.epd_w21_write_cmd(0x20)  # Activate Display Update Sequence
        self.lcd_chkstatus()

    def epd_update_fast(self):
        # Fast refresh 1 update function
        self.epd_w21_write_cmd(0x22)  # Display Update Control
        self.epd_w21_write_data(0xC7)
        self.epd_w21_write_cmd(0x20)  # Activate Display Update Sequence
        self.lcd_chkstatus()

    def epd_update_part(self):
        # Partial refresh update function
        self.epd_w21_write_cmd(0x22)  # Display Update Control
        self.epd_w21_write_data(0xFF)
        self.epd_w21_write_cmd(0x20)  # Activate Display Update Sequence
        self.lcd_chkstatus()

    # Clear screen functions - adapted from Arduino version
    def epd_clear_white(self):
        # Clear screen display - white
        self.epd_w21_write_cmd(0x24)  # write RAM for black(0)/white (1)
        for i in range(self.EPD_ARRAY):
            self.epd_w21_write_data(0xFF)
        self.epd_update()

    def epd_clear_black(self):
        # Display all black
        self.epd_w21_write_cmd(0x24)  # write RAM for black(0)/white (1)
        for i in range(self.EPD_ARRAY):
            self.epd_w21_write_data(0x00)
        self.epd_update()

    # Display functions adapted from Arduino version
    def EPD_Display(self, image):
        # Full screen refresh display function - adapted from EPD_WhiteScreen_ALL()
        self.epd_w21_write_cmd(0x24)  # write RAM for black(0)/white (1)
        for i in range(self.EPD_ARRAY):
            self.epd_w21_write_data(image[i])
        self.epd_update()

    def EPD_Display_Fast(self, image):
        # Fast refresh display function - adapted from EPD_WhiteScreen_ALL_Fast()
        self.epd_w21_write_cmd(0x24)  # write RAM for black(0)/white (1)
        for i in range(self.EPD_ARRAY):
            self.epd_w21_write_data(image[i])
        self.epd_update_fast()

    def EPD_Display_Fast_Dual(self, image):
        # Fast refresh with dual RAM write - adapted from EPD_WhiteScreen_ALL_Fast2()
        # This writes to both RAM buffers for better fast refresh performance
        self.epd_w21_write_cmd(0x24)  # write RAM for black(0)/white (1)
        for i in range(self.EPD_ARRAY):
            self.epd_w21_write_data(image[i])
        
        self.epd_w21_write_cmd(0x26)  # write RAM for black(0)/white (1)
        for i in range(self.EPD_ARRAY):
            self.epd_w21_write_data(0x00)
        
        self.epd_update_fast()

    # Partial refresh functions - adapted from Arduino version
    def epd_set_basemap(self, image_data):
        # Partial refresh of background display - adapted from EPD_SetRAMValue_BaseMap()
        self.epd_w21_write_cmd(0x24)  # Write Black and White image to RAM
        if isinstance(image_data, bytes):
            for byte in image_data:
                self.epd_w21_write_data(byte)
        else:
            for i in range(min(len(image_data), self.EPD_ARRAY)):
                self.epd_w21_write_data(image_data[i])
        
        self.epd_w21_write_cmd(0x26)  # Write Black and White image to RAM
        if isinstance(image_data, bytes):
            for byte in image_data:
                self.epd_w21_write_data(byte)
        else:
            for i in range(min(len(image_data), self.EPD_ARRAY)):
                self.epd_w21_write_data(image_data[i])
        
        self.epd_update()

    def epd_display_part_all(self, image_data):
        # Full screen partial refresh display - adapted from EPD_Dis_PartAll()
        # Module reset
        self.RST_PIN.low()
        self.delay_xms(10)
        self.RST_PIN.high()
        self.delay_xms(10)
        
        self.epd_w21_write_cmd(0x3C)  # BorderWavefrom
        self.epd_w21_write_data(0x80)

        self.epd_w21_write_cmd(0x24)  # Write Black and White image to RAM
        if isinstance(image_data, bytes):
            for byte in image_data:
                self.epd_w21_write_data(byte)
        else:
            for i in range(min(len(image_data), self.EPD_ARRAY)):
                self.epd_w21_write_data(image_data[i])
        
        self.epd_update_part()

    # Deep sleep function - adapted from EPD_DeepSleep()
    def epd_sleep(self):
        self.epd_w21_write_cmd(0x10)  # Enter deep sleep
        self.epd_w21_write_data(0x01)
        self.delay_xms(100)

    # Partial refresh initialization for animation sequences
    def epd_init_part(self):
        # For partial refresh, we don't need full re-initialization
        # Just ensure the display is ready for partial updates
        # The Arduino code shows partial refresh works after setting base map
        pass

    def power_off(self):
        # Power off - use deep sleep
        self.epd_sleep()

    def PIC_display(self, old_file_path, file_path):
        # Updated to use new Display_EPD_W21 style commands
        self.epd_w21_write_cmd(0x24)  # write RAM for black(0)/white (1)
        
        if file_path is not None:
            with open(file_path, 'rb') as file:
                byte = file.read(1)
                while byte:
                    self.epd_w21_write_data(ord(byte))
                    byte = file.read(1)
        else:
            for _ in range(self.EPD_ARRAY):
                self.epd_w21_write_data(0xFF)

        self.epd_update()

    def PIC_clear(self):
        # Clear screen - use white clear
        self.epd_clear_white()
    


# einkMux = machine.Pin(22, machine.Pin.OUT)
# einkStatus = machine.Pin(9, machine.Pin.OUT)
# einkMux.high()  # inverted logic
# einkStatus.high() # provide power to eink

# eink = einkDSP_SAM()
# # eink.epd_init()
# eink.epd_init_fast()
# eink.PIC_display(None, './loading1.bin')

# for i in range(1, 3):
#     eink.epd_init_part()
#     eink.PIC_display('./loading1.bin', './loading2.bin')
#     eink.epd_init_part()
#     eink.PIC_display('./loading2.bin', './loading1.bin')

# eink.PIC_display('./loading1.bin', './loading2.bin')
