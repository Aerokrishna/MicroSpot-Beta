#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
from threading import Thread

class InputDetector(Node):

    def __init__(self):
        super().__init__('input_detector')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)

    def start_keyboard_listener(self):
        self.keyboard_listener.start()

    def on_key_press(self, key):
        if hasattr(key, 'char'):
            key_str = key.char
        else:
            key_str = str(key)
        msg = String()
        msg.data = f"{key_str}"
        self.publisher_.publish(msg)
        print(f"Pressed key: {key_str}")

def main(args=None):
    rclpy.init(args=args)
    node = InputDetector()
    thread = Thread(target=node.start_keyboard_listener)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keyboard_listener.stop()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
