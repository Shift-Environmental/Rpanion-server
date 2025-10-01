# GCS Control System - User Guide

## Overview

The Rpanion server manages multiple Ground Control Station (GCS) connections to your vehicle, ensuring only one GCS has active control at any time. All connected GCS receive telemetry data, but only the **active controller** can send commands to the vehicle.

## How Active Control is Determined

### First Connected = Active Controller
The **first GCS to connect** becomes the active controller, regardless of System ID.

### System ID Priority (Fallback Only)
System ID priority is only used when the active controller disconnects or relinquishes control:

- **Higher System ID = Higher Priority** for fallback
- When the active GCS disconnects, the backup GCS with the highest System ID takes over
- A newly connected GCS with a high System ID will NOT automatically take control from an active GCS
- Priority only matters when selecting a new active controller

### Example Control Scenario
```
Initial State:
- GCS System ID 100 connects first → ACTIVE CONTROLLER ✓

GCS System ID 255 connects:
- System ID 100 → Still ACTIVE CONTROLLER ✓
- System ID 255 → Backup (receives telemetry only)

System ID 100 disconnects:
- System ID 255 → Now ACTIVE CONTROLLER ✓ (highest System ID)

System ID 100 reconnects:
- System ID 255 → Still ACTIVE CONTROLLER ✓
- System ID 100 → Now backup (receives telemetry only)
```

## Connection Requirements

### One GCS Per System ID
- Each GCS must have a **unique System ID**
- If two GCS attempt to connect with the same System ID, only the first one is accepted
- The second connection will be rejected to prevent conflicts

### Heartbeat Monitoring
- All GCS must send heartbeats regularly (default timeout: 5 seconds)
- If the active controller stops sending heartbeats, it's automatically removed
- The next highest-priority GCS immediately takes control
- Backup GCS that timeout are removed from the system

## What Each GCS Receives

### Active Controller
✓ Receives all telemetry data  
✓ Can send all commands (arm/disarm, mode changes, waypoints, parameters, etc.)  
✓ Full vehicle control

### Backup GCS (Not Active)
✓ Receives all telemetry data  
✗ Commands are **blocked** (not forwarded to vehicle)  
✗ Cannot control the vehicle

## Relinquishing Control

The active controller can voluntarily give up control by sending the **Relinquish Control** command (MAVLink command 45000).

### What Happens When You Relinquish:
1. Your GCS connection is removed from the system
2. The backup GCS with the highest System ID becomes active
3. Your next heartbeat will reconnect you as a new connection (as a backup, unless no other GCS exists)

### Special Case - Only One GCS:
If you're the only connected GCS and relinquish control:
- You'll be removed from the system
- On your next heartbeat, you'll reconnect and regain control automatically (since no other GCS exists)
- This acts as a "soft reset" of your connection

## Troubleshooting

### My commands aren't working
- Check if you're the active controller (first to connect, or highest System ID after previous controller disconnected)
- Verify your GCS is sending heartbeats regularly
- If another GCS connected first, they have control until they disconnect or relinquish

### I can't connect
- Verify your System ID is unique (not already in use by another GCS)
- Check network connectivity to the Rpanion server
- Confirm the UDP port is correct

### Control keeps switching
- Multiple GCS are connecting/disconnecting rapidly
- Check that the active controller has a stable network connection
- If the active controller keeps timing out, backup GCS will repeatedly take over based on System ID priority

## Technical Details

- **Heartbeat timeout**: 5 seconds (configurable)
- **Relinquish command**: MAV_CMD 45000
- **GCS detection**: MAVLink types 6, 18, 27
- **Priority method**: Highest System ID wins
- **Monitoring interval**: Every 2 seconds