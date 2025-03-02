#!/usr/bin/env python3
import evdev
from evdev import ecodes, InputDevice, ff
import time
import sys

def main():
    # List all input devices
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    
    # Print all devices for debugging
    print("Available devices:")
    for i, device in enumerate(devices):
        print(f"{i}: {device.path}, {device.name}, {device.phys}")
    
    # Ask user to select a device
    if len(devices) == 0:
        print("No input devices found")
        return
    
    device_idx = 0
    if len(devices) > 1:
        device_idx = int(input("Select device by index: "))
    
    device = devices[device_idx]
    print(f"Selected device: {device.name}")
    
    # Check if device supports force feedback
    if ecodes.EV_FF not in device.capabilities():
        print("Device does not support force feedback")
        return
    
    print("Device supports force feedback")
    print("FF capabilities:", device.capabilities()[ecodes.EV_FF])
    
    # Send autocenter command
    try:
        # First, set autocenter to 0 (off)
        print("Setting autocenter to 0 (off)")
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        time.sleep(2)
        
        # Then set autocenter to 50% strength
        print("Setting autocenter to 50% strength")
        autocenter_strength = 0xFFFF * 50 // 100  # 50% of max strength
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, autocenter_strength)
        time.sleep(2)
        
        # Then set autocenter to 100% strength
        print("Setting autocenter to 100% strength")
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0xFFFF)
        time.sleep(100)
        
        # Finally, turn it off again
        print("Setting autocenter back to 0 (off)")
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        
    except Exception as e:
        print(f"Error sending force feedback: {e}")
    finally:
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
    print("Done")

if __name__ == "__main__":
    print('autocenter')
    main()
