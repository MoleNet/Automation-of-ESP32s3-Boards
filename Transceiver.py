import machine
import utime
from sx127x import Transceiver

spi  = machine.SoftSPI(baudrate=400_000, sck=14, mosi=47, miso=21)
cs   = machine.Pin(48, machine.Pin.OUT, value=1)
rst  = machine.Pin(45, machine.Pin.OUT, value=1)
dio0 = machine.Pin(46, machine.Pin.IN)

rx = Transceiver(spi, cs, rst, dio0)

rx.settings(
    power=14,
    sf=7,
    bw=125,
    cr=4/5,
    syn_word=0x12,
    inv_iq=False,
    crc=True,
    exp_header=True
)

print("LoRa receiver ready")

while True:
    msg, snr, rssi = rx.receive(868.1, timeout=10)

    if msg:
        try:
            text = msg.decode()
        except:
            text = str(msg)

        print("LoRa RX →", text)
        print("SNR:", snr, "RSSI:", rssi)
    else:
        print("No packet")

    utime.sleep(0.2)

