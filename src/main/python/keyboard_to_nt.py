import keyboard
import ntcore
import threading
import time

ntcoreinst = ntcore.NetworkTableInstance.getDefault()

print("Setting up NetworkTables client")
ntcoreinst.startClient4("KeyboardToNT")
ntcoreinst.setServer("127.0.0.1")
ntcoreinst.startDSClient()

# Wait for connection
print("Waiting for connection to NetworkTables server...")
while not ntcoreinst.isConnected():
    time.sleep(0.1)

table = ntcoreinst.getTable("/SmartDashboard/keyboard")

print("Connected!")

minimum_press_time = 0.35
keys_dict = {}
lock = threading.Lock()


def turn_off_keys_with_delay():
    while True:
        time.sleep(0.01)
        lock.acquire()
        for key in keys_dict:
            if (keys_dict[key][1] and time.time() - keys_dict[key][0] > minimum_press_time):
                table.putBoolean(key, False)
                keys_dict.pop(key, None)
                break
        lock.release()


def on_action(event: keyboard.KeyboardEvent):
    if event == None or event.name == None or event.name == "/":
        return

    key = ""
    if event.is_keypad:
        key = "numpad" + event.name
    else:
        key = event.name.lower()

    isPressed = event.event_type == keyboard.KEY_DOWN

    if isPressed:
        table.putBoolean(key, True)
        lock.acquire()
        if key not in keys_dict:
            keys_dict[key] = [time.time(), False]
        lock.release()
        return

    lock.acquire()
    if key in keys_dict:
        keys_dict[key][1] = True
    else:
        keys_dict[key] = [time.time(), False]
    lock.release()


def main():
    keyboard.hook(on_action)
    thread = threading.Thread(turn_off_keys_with_delay())
    thread.start()
    keyboard.wait()


main()
