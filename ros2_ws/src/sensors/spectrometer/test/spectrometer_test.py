import time
import board
import busio
from adafruit_as7341 import AS7341

# Initialize I2C bus and sensor
i2c = busio.I2C(board.SCL_1, board.SDA_1)
sensor = AS7341(i2c)

def application_task():
    while True:
        print("--- Spectrometer Readings ---")
        
        # Equivalent to Mode 1 (F1-F4)
        try:
            print(f"F1 (415nm, FWHM 26nm)  : {sensor.channel_415nm}")
        except:
            print(f"F1 (415nm, FWHM 26nm)  : NA")
        try:
            print(f"F2 (445nm, FWHM 30nm) : {sensor.channel_445nm}")
        except:
            print(f"F2 (445nm, FWHM 30nm) : NA")
        try:
            print(f"F3 (480nm, FWHM 36nm) : {sensor.channel_480nm}")
        except:
            print(f"F3 (480nm, FWHM 36nm) : NA")
        try:
            print(f"F4 (515nm, FWHM 39nm) : {sensor.channel_515nm}")
        except:
            print(f"F4 (515nm, FWHM 39nm) : NA")
        
        # Equivalent to Mode 2 (F5-F8)
        try:
            print(f"F5 (555nm, FWHM 39nm) : {sensor.channel_555nm}")
        except:
            print(f"F5 (555nm, FWHM 39nm) : NA")
        try:
            print(f"F6 (590nm, FWHM 40nm) : {sensor.channel_590nm}")
        except:
            print(f"F6 (590nm, FWHM 40nm) : NA")
        try:
            print(f"F7 (630nm, FWHM 50nm) : {sensor.channel_630nm}")
        except:
            print(f"F7 (630nm, FWHM 50nm) : NA")
        try:
            print(f"F8 (680nm, FWHM 52nm) : {sensor.channel_680nm}")
        except:
            print(f"F8 (680nm, FWHM 52nm) : NA")
        
        # Clear / NIR channels
        print(f"Clear (790nm, FWHM 52nm)  : {sensor.channel_clear}")
        print(f"NIR   (920nm, FWHM 100nm) : {sensor.channel_nir}")
        
        # Note on Flicker Detection: 
        # The base adafruit library handles spectral reading easily. 
        # Advanced features like flicker detection (100Hz/120Hz) require 
        # reading specific flicker registers directly via sensor._read_register(), 
        # but the core spectral channels are completely covered above.
        
        print("---------------------------\n")
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        application_task()
    except KeyboardInterrupt:
        print("\nExiting program.")