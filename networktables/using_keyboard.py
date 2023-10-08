import time
import ntcore # pip install robotpy
import keyboard # pip install keyboard

inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("keyboard")
inst.startClient4("keyboard")

# pick one of these
# inst.setServer("10.te.am.2")
# inst.setServerTeam(4096)
# inst.startDSClient()

while not inst.isConnected():
    print("not connected")
    time.sleep(1)

while True:
    event = keyboard.read_event()
    if event.name:
        table.putBoolean(event.name, event.event_type == keyboard.KEY_DOWN)