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

def apply_force_in_direction(device, force_direction, strength=100, duration=2):
    """Apply a constant force to the wheel in a specific direction
    
    Args:
        device: The input device
        force_direction: Force direction from -100 (full left) to 100 (full right), 0 is no force
        strength: Force strength as percentage (0-100)
        duration: How long to apply the force in seconds
    
    The wheel will move in the specified direction while the force is applied,
    but there is no guarantee it will stop at any specific position.
    """
    try:
        # Validate force_direction range
        force_direction = max(-100, min(100, force_direction))
        
        # Calculate direction and level
        # For constant effects, direction determines the direction of the force
        # 0 degrees = left, 90 degrees = down, 180 degrees = right, 270 degrees = up
        # We'll use 0 for left and 0xC000 (180 degrees) for right
        direction = 0xC000 if force_direction > 0 else -1
        level = int(abs(force_direction) * 32767 / 100)  # Scale to 0-32767 range
        
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
        print(f"Applying {'right' if force_direction > 0 else 'left'} force of {abs(force_direction)}% with {strength}% strength for {duration}s")
        
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
        print(f"Error applying force: {e}")
        return False

def stop_all_forces(device):
    """Stop all force feedback effects on the device
    
    This function will:
    1. Turn off autocenter
    2. Send a STOP_FORCE command to stop all active effects
    """
    try:
        print("Stopping all force feedback effects...")
        # Turn off autocenter
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        
        # Send STOP_ALL_FORCES (FF_STOP_ALL in evdev)
        # This will stop all effects that are currently playing
        device.write(ecodes.EV_FF, ecodes.FF_STOP_ALL, 0)
        
        print("All forces stopped")
        return True
    except Exception as e:
        print(f"Error stopping forces: {e}")
        return False

def apply_damper_effect(device, strength=100, duration=2):
    """Apply a damper effect to the wheel
    
    Args:
        device: The input device
        strength: Effect strength as percentage (0-100)
        duration: How long to apply the effect in seconds
    """
    try:
        # Create damper effect
        effect = ff.Effect(
            ecodes.FF_DAMPER,   # effect type
            -1,                 # effect id (assigned by device)
            0,                  # direction (not used for damper)
            ff.Trigger(0, 0),   # no trigger
            ff.Replay(int(duration * 1000), 0),  # duration in ms, delay
            ff.EffectType()     # empty effect type as damper doesn't need additional parameters
        )
        
        # Upload and play the effect
        effect_id = device.upload_effect(effect)
        print(f"Applying damper effect with {strength}% strength for {duration}s")
        
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
        print(f"Error applying damper effect: {e}")
        return False

def apply_friction_effect(device, strength=100, duration=2):
    """Apply a friction effect to the wheel
    
    Args:
        device: The input device
        strength: Effect strength as percentage (0-100)
        duration: How long to apply the effect in seconds
    """
    try:
        # Create friction effect
        effect = ff.Effect(
            ecodes.FF_FRICTION, # effect type
            -1,                 # effect id (assigned by device)
            0,                  # direction (not used for friction)
            ff.Trigger(0, 0),   # no trigger
            ff.Replay(int(duration * 1000), 0),  # duration in ms, delay
            ff.EffectType()     # empty effect type as friction doesn't need additional parameters
        )
        
        # Upload and play the effect
        effect_id = device.upload_effect(effect)
        print(f"Applying friction effect with {strength}% strength for {duration}s")
        
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
        print(f"Error applying friction effect: {e}")
        return False

def test_autocenter(device):
    """Test autocenter functionality"""
    print("Testing autocenter...")
    
    # Turn off autocenter
    set_autocenter(device, 0)
    time.sleep(1)
    
    # Move wheel to the right
    print("Applying force to the right...")
    apply_force_in_direction(device, 100, 100, 2)
    time.sleep(1)
    
    # Move wheel to the left
    print("Applying force to the left...")
    apply_force_in_direction(device, -100, 100, 2)
    time.sleep(1)
    
    # Turn on autocenter
    print("Turning on autocenter...")
    set_autocenter(device, 100)
    time.sleep(3)
    
    # Turn off autocenter
    print("Turning off autocenter...")
    set_autocenter(device, 0)

def test_damper(device):
    """Test damper functionality"""
    print("Testing damper effect...")
    
    # Apply damper effect at different strengths
    for strength in [25, 50, 75, 100]:
        print(f"\nTesting {strength}% strength...")
        apply_damper_effect(device, strength, 3)
        time.sleep(1)

def test_friction(device):
    """Test friction functionality"""
    print("Testing friction effect...")
    
    # Apply friction effect at different strengths
    for strength in [25, 50, 75, 100]:
        print(f"\nTesting {strength}% strength...")
        apply_friction_effect(device, strength, 3)
        time.sleep(1)

def test_range(device):
    """Test range functionality"""
    print("Testing wheel range settings...")
    
    # Test different range values
    test_ranges = [180, 360, 540, 900]
    for range_value in test_ranges:
        print(f"\nSetting range to {range_value} degrees...")
        set_range(device, range_value)
        print("Try turning the wheel to feel the difference")
        time.sleep(5)
    
    # Reset to a common default (540 degrees)
    print("\nResetting to default range (540 degrees)...")
    set_range(device, 540)

def test_constant_force(device):
    """Test constant force functionality"""
    print("Testing constant force effects...")
    
    # Make sure autocenter is off
    set_autocenter(device, 0)
    time.sleep(1)
    
    # Test different directions and strengths
    directions = [
        (100, "right, full strength"),
        (50, "right, half strength"),
        (-50, "left, half strength"),
        (-100, "left, full strength")
    ]
    
    for force_direction, description in directions:
        print(f"\nApplying constant force to the {description}...")
        apply_force_in_direction(device, force_direction, 100, 3)
        time.sleep(1)
    
    # Test different strength levels in one direction
    print("\nTesting different strength levels (right direction)...")
    for strength in [25, 50, 75, 100]:
        print(f"Applying {strength}% strength...")
        apply_force_in_direction(device, 100, strength, 2)
        time.sleep(1)
    
    # Reset by stopping all forces
    stop_all_forces(device)

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
    
    # Stop command
    stop_parser = subparsers.add_parser('stop', help='Stop all force feedback effects')
    
    # Test command
    test_parser = subparsers.add_parser('test', help='Run predefined tests')
    test_parser.add_argument('test_name', choices=['autocenter', 'damper', 'friction', 'range', 'constant'], 
                           help='Test to run')
    
    # Add new effect parsers
    damper_parser = subparsers.add_parser('damper', help='Apply damper effect')
    damper_parser.add_argument('--strength', '-s', type=int, default=100,
                             help='Effect strength (0-100)')
    damper_parser.add_argument('--duration', '-d', type=float, default=2,
                             help='Duration in seconds')
    
    friction_parser = subparsers.add_parser('friction', help='Apply friction effect')
    friction_parser.add_argument('--strength', '-s', type=int, default=100,
                               help='Effect strength (0-100)')
    friction_parser.add_argument('--duration', '-d', type=float, default=2,
                               help='Duration in seconds')
    
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
            apply_force_in_direction(device, args.position, args.strength, args.duration)
        
        elif args.command == 'damper':
            apply_damper_effect(device, args.strength, args.duration)
        
        elif args.command == 'friction':
            apply_friction_effect(device, args.strength, args.duration)
        
        elif args.command == 'stop':
            stop_all_forces(device)
        
        elif args.command == 'test':
            if args.test_name == 'autocenter':
                test_autocenter(device)
            elif args.test_name == 'damper':
                test_damper(device)
            elif args.test_name == 'friction':
                test_friction(device)
            elif args.test_name == 'range':
                test_range(device)
            elif args.test_name == 'constant':
                test_constant_force(device)
    
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
