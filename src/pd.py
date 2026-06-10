from smbus2 import SMBus
import MCP342x
i2cbus = SMBus(1)
MCP3424_fiber=MCP342x.MCP342x(i2cbus, 0x68, device='MCP3424', channel=0, gain=1, resolution=16, continuous_mode=False, scale_factor=1.0, offset=0.0)
# MCP3424_pinhole=MCP342x.MCP342x(i2cbus, 0x68, device='MCP3424', channel=1, gain=1, resolution=16, continuous_mode=False, scale_factor=1.0, offset=0.0)
# MCP3424_ref=MCP342x.MCP342x(i2cbus, 0x68, device='MCP3424', channel=2, gain=4, resolution=16, continuous_mode=False, scale_factor=1.0, offset=0.0)

# ADS1115_fiber
# from adafruit_ads1x15.analog_in import AnalogIn
# import adafruit_ads1x15.ads1115 as ADS1115
# import adafruit_ads1x15
# import board
# import busio
# import time

# # Create the I2C bus interface
# i2c = busio.I2C(board.SCL, board.SDA)

# # Create the ADS1115 instance
# ads = ADS1115.ADS1115(i2c)
# # PDA8A with 50 ohm load, will be 0 - 1.8 V
# # ADS1115.PGA_2: ±2.048V
# ads.gain = 4
# ads.data_rate = 860

# # Create analog input channels
# ADS1115_fiber = AnalogIn(ads, ADS1115.P0)  # Channel 0

# print(f"Fiber channel reading: {ADS1115_fiber.value}")

# t0 = time.time()
# for i in range(100):
#     # print(f"Fiber channel reading: {ADS1115_fiber.value}")
#     a= ADS1115_fiber.value
# t1 = time.time()
# print(f"Time elapsed: {t1-t0}")