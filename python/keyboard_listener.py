from pynput import keyboard


class KeyboardListener:
    def __init__(self):
        self.key = ""
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        try:
            if key.char.isalnum():
                self.key = key.char
        except AttributeError:
            pass
