import time
from inputs import get_gamepad

def test_gamepad():
    print("Reading gamepad inputs...")
    while True:
        events = get_gamepad()
        for event in events:
            print("Event code: {}, State: {}".format(event.code, event.state))
        time.sleep(0.1)

if __name__ == "__main__":
    test_gamepad()