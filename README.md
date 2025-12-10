# SNPX Client

A Python 3 communication library for reading and writing data to Fanuc Robots with HMI Device (SNPX) Option R553. This library provides a clean, object-oriented interface for interacting with Fanuc robot controllers via the SNPX protocol.

## Features

- **Digital I/O Support**: Read and write digital inputs/outputs (DI, DO, UI, UO, SI, SO)
- **Position Data**: Read joint positions and Cartesian positions from the robot
- **System Variables**: Read and write system variables (INT, REAL). STRING types still WIP
- **Automatic Assignment Management**: Automatically manages SNPX assignments with size-aware allocation
- **Clean API**: Simple, intuitive interface for robot communication

## Installation

No external dependencies required - uses only Python standard library:
- `socket`
- `struct`
- `time`
- `math`
- `enum`
- `dataclasses`

## Quick Start

```python
from snpx_client import SnpxClient, VariableTypes

# Connect to robot
client = SnpxClient(ip="192.168.1.100", connect_on_init=True)

# Read joint positions
joints = client.j_pos.read()
print(f"Joint positions: {joints}")

# Read digital outputs
outputs = client.do.read(count=64, start_index=1)
print(f"Digital outputs: {outputs}")

# Write to digital inputs
client.di.write([True, False, True], start_index=1)

# Read system variable
angle_tolerance = client.read_sys_var("$ANGTOL[1]", VariableTypes.REAL)
print(f"Angle tolerance: {angle_tolerance}")

# Write system variable
client.write_sys_var("$ANGTOL[1]", VariableTypes.REAL, 1200.13)

# Disconnect
client.disconnect()
```

## Usage Examples

### Digital I/O Operations

```python
# Read digital outputs
outputs = client.do.read(count=32, start_index=1)

# Write to digital inputs
client.di.write([True] * 16 + [False] * 16, start_index=1)

# Read user inputs/outputs
user_inputs = client.ui.read(count=16, start_index=1)
client.uo.write([True, False, True], start_index=1)

# Read/write SOP signals
sop_inputs = client.si.read(count=8, start_index=1)
client.so.write([False] * 8, start_index=1)
```

### Position Data

```python
# Read joint positions (returns list of floats)
joints = client.j_pos.read()

# Read Cartesian position
cartesian = client.cart_pos.read()
```

### System Variables

The library supports three variable types:
- `VariableTypes.INT`: Integer values
- `VariableTypes.REAL`: Floating-point values
- `VariableTypes.STRING`: String values

```python
# Read system variables
int_var = client.read_sys_var("$AC_CRC_ACCO[1]", VariableTypes.INT)
real_var = client.read_sys_var("$ANGTOL[1]", VariableTypes.REAL)


# Write system variables
client.write_sys_var("$ANGTOL[1]", VariableTypes.REAL, 1200.13)
client.write_sys_var("$SCR_GRP[1].$MCR_NAME", VariableTypes.STRING, "MyProgram")
```

### Manual Assignment Management

```python
# Manually set an assignment (optional - usually automatic)
client.set_asg("$ANGTOL[1]", VariableTypes.REAL, asg_num=1)

# Check if assignment number is available
is_available = client.check_if_asg_avail(1, size=2)

# Get next available assignment number
next_asg = client.get_next_asg_num(size=50)
```

## API Reference

### SnpxClient

Main client class for robot communication.

#### Methods

- `connect()`: Establish connection to robot controller
- `disconnect()`: Close connection
- `init_signals()`: Initialize signal/memory objects (called automatically on connect)
- `read_sys_var(var_name, var_type)`: Read a system variable
- `write_sys_var(var_name, var_type, value)`: Write a system variable
- `set_asg(var_name, var_type, asg_num=None)`: Set variable assignment
- `check_if_asg_avail(num, size=1)`: Check if assignment number is available
- `get_next_asg_num(size=1)`: Get next available assignment number

#### Attributes

After `init_signals()`, the following attributes are available:
- `di`: Digital inputs
- `do`: Digital outputs
- `ui`: User inputs
- `uo`: User outputs
- `si`: SOP inputs
- `so`: SOP outputs
- `j_pos`: Joint position data
- `cart_pos`: Cartesian position data

### DigitalSignal

Class for reading/writing digital signals.

#### Methods

- `read(count, start_index=1)`: Read boolean values
- `write(value, start_index=1)`: Write boolean values

### PositionData

Class for reading position data from the robot.

#### Methods

- `read()`: Read position data (returns list of floats)

## Assignment Management

The library automatically manages SNPX assignments when reading/writing system variables. It:
- Tracks assignment numbers and sizes
- Prevents overlapping assignments
- Automatically finds available assignment slots
- Reuses existing assignments for the same variable

Assignment numbers range from 1-80, and the system considers variable sizes to prevent conflicts. For example, if a variable at index 1 has size 50, the next available index will be 51.

## Protocol Details

This library implements the SNPX (SNP Extended) protocol for Fanuc robots. The protocol is based on the GE SRTP protocol, adapted for Fanuc's SNPX implementation.

Packet structure follows the standard SNPX format with:
- 56-byte base message header
- Variable-length payloads for commands and data
- Little-endian byte ordering
- ASCII command strings for SETASG operations

## Requirements

- Python 3.6+
- Fanuc robot with SNPX Option R553 enabled
- Network connectivity to robot controller (default port: 60008)

## Notes

- The library handles connection initialization automatically
- Assignment numbers are managed internally - you typically don't need to set them manually
- Digital I/O operations support variable bit counts
- Position data reading returns joint values as floats
- System variable read/write operations automatically handle assignment creation

## Credits

This project was inspired by and built upon knowledge from the [Fanuc_GESRTP_Driver](https://github.com/Booozie-Z/Fanuc_GESRTP_Driver) repository by [Booozie-Z](https://github.com/Booozie-Z). The base protocol understanding and packet structure insights came from that excellent reference implementation.

## License

This project is provided as-is for use with Fanuc robot controllers.

## Contributing

Contributions, issues, and feature requests are welcome!

## Disclaimer

This software is not affiliated with or endorsed by Fanuc Corporation. Use at your own risk.

