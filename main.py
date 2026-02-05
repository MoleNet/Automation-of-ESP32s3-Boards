import machine
import utime

# BME280 sensor driver
from BME280 import BME280  

# SX1276 LoRa driver (Transceiver class)
from sx127x import Transceiver

# I²C bus for BME280
i2c = machine.SoftI2C(
    scl=machine.Pin(8),  
    sda=machine.Pin(9),
    freq=100_000
)
print("Scanning I²C ")
found = i2c.scan()
if not found:
    print("No I²C devices detected – check wiring/pull‑ups!")
else:
    print(" Devices:", ["0x{:02X}".format(a) for a in found])
utime.sleep_ms(10)

# Initialise BME280
try:
    bme = BME280(i2c=i2c, address=0x76)          # default address 0x76
    print("BME280: OK")
except Exception as e:
    bme = None
    print("BME280 init error →", e)

# SPI bus and pins for SX1276
spi  = machine.SoftSPI(baudrate=400_000, sck=14, mosi=47, miso=21)
cs   = machine.Pin(48, machine.Pin.OUT, value=1)   # chip‑select
rst  = machine.Pin(45, machine.Pin.OUT, value=1)   # reset
dio0 = machine.Pin(46, machine.Pin.IN)            # DIO0 interrupt

# Initialise LoRa transceiver
tx = Transceiver(spi, cs, rst, dio0)

# Apply the same radio configuration you used before
tx.settings(
    power=14,          
    sf=7,
    bw=125,
    cr=4/5,
    syn_word=0x12,
    inv_iq=False,
    crc=True,
    exp_header=True
)

# Main loop – read sensor, build payload, send

while True:
    if bme:
      #         payload = "{:.1f},{:.1f},{:.0f}".format(bme.temperature,bme.humidity,bme.pressure)

        payload = f"temp:{bme.temperature},hum:{bme.humidity},press:{bme.pressure}"
        print("BME280:", payload)
    else:
        payload = "no_sensor"
        print("noBME280:", payload)

    try:
        # Transmit on 868.1 MHz
        tx.send(payload, freq=868.1)
    except Exception as e:
        print("LoRa send error →", e)

    # Wait a short interval before the next transmission
    utime.sleep_ms(2000)

