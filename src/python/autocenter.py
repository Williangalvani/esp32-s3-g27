#!/usr/bin/env python3
import evdev
from evdev import ecodes, InputDevice, ff
import time
import sys
import argparse

def select_device():
    # List all input devices
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    
    # Print all devices for debugging
    print("Available devices:")
    for i, device in enumerate(devices):
        print(f"{i}: {device.path}, {device.name}, {device.phys}")
    
    # Ask user to select a device
    if len(devices) == 0:
        print("No input devices found")
        return None
    
    device_idx = 0
    if len(devices) > 1:
        device_idx = int(input("Select device by index: "))
    
    device = devices[device_idx]
    print(f"Selected device: {device.name}")
    
    # Check if device supports force feedback
    if ecodes.EV_FF not in device.capabilities():
        print("Device does not support force feedback")
        return None
    
    print("Device supports force feedback")
    
    # Get FF capabilities and print them in a human-readable format
    ff_caps = device.capabilities()[ecodes.EV_FF]
    print("FF capabilities:", ff_caps)
    
    # Print human-readable FF capabilities
    ff_capability_names = {
        ecodes.FF_CONSTANT: "Constant Force",
        ecodes.FF_PERIODIC: "Periodic Effect",
        ecodes.FF_RAMP: "Ramp Force",
        ecodes.FF_SPRING: "Spring",
        ecodes.FF_FRICTION: "Friction",
        ecodes.FF_DAMPER: "Damper",
        ecodes.FF_RUMBLE: "Rumble",
        ecodes.FF_INERTIA: "Inertia",
        ecodes.FF_GAIN: "Gain",
        ecodes.FF_AUTOCENTER: "Autocenter"
    }
    
    print("Supported effects:")
    for cap in ff_caps:
        if cap in ff_capability_names:
            print(f"  - {ff_capability_names[cap]} (code: {cap})")
        else:
            print(f"  - Unknown capability (code: {cap})")
    
    return device

def set_autocenter(device, strength, duration=2):
    """Set autocenter with specified strength (0-100)"""
    try:
        # Convert percentage to actual value (0 to 0xFFFF)
        autocenter_value = 0xFFFF * strength // 100
        print(f"Setting autocenter to {strength}% strength")
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, autocenter_value)
        if duration > 0:
            time.sleep(duration)
        return True
    except Exception as e:
        print(f"Error setting autocenter: {e}")
        return False

def move_to_position(device, position, strength=100, duration=2):
    """Move wheel to a specific position (-100 to 100, where 0 is center)"""
    try:
        # Validate position range
        position = max(-100, min(100, position))
        
        # Calculate direction and level
        # For constant effects, direction determines the direction of the force
        # 0 degrees = left, 90 degrees = down, 180 degrees = right, 270 degrees = up
        # We'll use 0 for left and 0xC000 (180 degrees) for right
        direction = 0xC000 if position > 0 else 0
        level = int(abs(position) * 32767 / 100)  # Scale to 0-32767 range
        
        # Create envelope and constant effect
        envelope = ff.Envelope(500, 32767, 500, 32767)
        constant = ff.Constant(level, envelope)
        
        # Create the effect
        effect = ff.Effect(
            ecodes.FF_CONSTANT,  # effect type
            -1,                  # effect id (assigned by device)
            direction,           # direction
            ff.Trigger(0, 0),    # no trigger
            ff.Replay(int(duration * 1000), 0),  # duration in ms, delay
            ff.EffectType(ff_constant_effect=constant)
        )
        
        # Upload the effect to the device
        effect_id = device.upload_effect(effect)
        print(f"Moving wheel to position {position} with {strength}% strength")
        
        # Set the gain (strength)
        device.write(ecodes.EV_FF, ecodes.FF_GAIN, 0xFFFF * strength // 100)
        
        # Play the effect
        device.write(ecodes.EV_FF, effect_id, 1)
        
        # Wait for the specified duration
        if duration > 0:
            time.sleep(duration)
            
        # Stop and remove the effect
        device.write(ecodes.EV_FF, effect_id, 0)
        device.erase_effect(effect_id)
        return True
    except Exception as e:
        print(f"Error moving wheel: {e}")
        return False

def test_autocenter(device):
    """Run the original autocenter test sequence"""
    try:
        # First, set autocenter to 0 (off)
        set_autocenter(device, 0)
        time.sleep(2)
        
        # Then set autocenter to 50% strength
        set_autocenter(device, 50)
        time.sleep(2)
        
        # Then set autocenter to 100% strength
        set_autocenter(device, 100)
        time.sleep(2)
        
        # Finally, turn it off again
        set_autocenter(device, 0)
        
    except Exception as e:
        print(f"Error in autocenter test: {e}")
    finally:
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
    print("Autocenter test completed")

def main():
    parser = argparse.ArgumentParser(description='Force feedback testing tool')
    
    # Create subparsers for different commands
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Autocenter command
    autocenter_parser = subparsers.add_parser('autocenter', help='Set autocenter strength')
    autocenter_parser.add_argument('strength', type=int, help='Autocenter strength (0-100)')
    autocenter_parser.add_argument('--duration', '-d', type=float, default=0, 
                                  help='Duration in seconds (0 for indefinite)')
    
    # Position command
    position_parser = subparsers.add_parser('position', help='Move wheel to position')
    position_parser.add_argument('position', type=int, 
                               help='Target position (-100 to 100, where 0 is center)')
    position_parser.add_argument('--strength', '-s', type=int, default=100,
                               help='Effect strength (0-100)')
    position_parser.add_argument('--duration', '-d', type=float, default=2,
                               help='Duration in seconds')
    
    # Test command
    test_parser = subparsers.add_parser('test', help='Run predefined tests')
    test_parser.add_argument('test_name', choices=['autocenter'], 
                           help='Test to run')
    
    # Parse arguments
    args = parser.parse_args()
    
    # If no command is provided, show help
    if not args.command:
        parser.print_help()
        return
    
    # Select device
    device = select_device()
    if not device:
        return
    
    try:
        # Execute the requested command
        if args.command == 'autocenter':
            set_autocenter(device, args.strength, args.duration)
        
        elif args.command == 'position':
            move_to_position(device, args.position, args.strength, args.duration)
        
        elif args.command == 'test':
            if args.test_name == 'autocenter':
                test_autocenter(device)
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Always turn off autocenter when done
        try:
            device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        except:
            pass
    
    print("Done")

if __name__ == "__main__":
    main()
