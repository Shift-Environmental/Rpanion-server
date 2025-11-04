# GCS Control System - User Guide

## Overview

The Rpanion server manages multiple Ground Control Station (GCS) connections to your vehicle, ensuring only one GCS has active control at any time. All connected GCS receive telemetry data, but only the **active controller** can send commands to the vehicle.

## How Active Control is Determined

### First Connected = Active Controller
The **first GCS to connect** becomes the active controller, regardless of System ID.

### System ID Priority (Fallback Only)
System ID priority is only used when the active controller disconnects or relinquishes control:

- **Lower System ID = Higher Priority** for fallback
- When the active GCS disconnects, the backup GCS with the lowest System ID takes over
- A newly connected GCS with a lower System ID will NOT automatically take control from an active GCS

## Connection Requirements

### One GCS Per System ID
- Each GCS must have a **unique System ID**
- If two GCS attempt to connect with the same System ID, only the first one is accepted
- The second connection will be rejected to prevent conflicts

### Heartbeat Monitoring
- All GCS must send heartbeats regularly (default timeout: 5 seconds)
- If the active controller stops sending heartbeats, it's automatically removed
- The backup GCS with the lowest System ID immediately takes control
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

The active controller can voluntarily give up control by sending the **Relinquish Control** command

The Relinquish Control command is sent as a MAVLink `COMMAND_LONG` message (message ID 76). The COMMAND_LONG message is a standard MAVLink message type used to send various commands, with a command field specifying the action. In this case, the command field is set to `45000`, which uniquely represents `MAV_CMD_RELINQUISH_CONTROL` in mavManager.js.

### What Happens When You Relinquish:
1. Your GCS connection is removed from the system
2. The backup GCS with the lowest System ID becomes active
3. Your next heartbeat will reconnect you as a new connection (as a backup, unless no other GCS exists)

### Special Case - Only One GCS:
If you're the only connected GCS and relinquish control:
- You'll be removed from the system
- On your next heartbeat, you'll reconnect and regain control automatically (since no other GCS exists)
- This acts as a "soft reset" of your connection

The broker **fully supports missions and waypoints**. Let me show you exactly how it works:

## ✅ Mission/Waypoint Support

The broker handles all mission-related messages correctly:

### **Active Controller** (Full Mission Access)
✅ Upload waypoints
✅ Download waypoints  
✅ Clear mission
✅ Set current waypoint
✅ Request mission list
✅ Partial mission updates

### **Backup GCS** (Limited Access)
❌ Cannot upload/modify waypoints (blocked)
❌ Cannot clear mission (blocked)
❌ Cannot set current waypoint (blocked)
✅ **CAN** request mission download (read-only)
✅ **CAN** monitor mission progress (telemetry)

## 📋 Mission Messages Handled

The broker already blocks these mission command messages for non-active GCS:

```python
COMMAND_MESSAGE_IDS = {
    # ... other commands ...
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM,            # 39 - Upload waypoint
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT,        # 73 - Upload waypoint (INT)
    mavlink.MAVLINK_MSG_ID_MISSION_COUNT,           # 44 - Start mission upload
    mavlink.MAVLINK_MSG_ID_MISSION_CLEAR_ALL,       # 45 - Clear all waypoints
    mavlink.MAVLINK_MSG_ID_MISSION_SET_CURRENT,     # 41 - Set active waypoint
    mavlink.MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST, # Partial update
}
```

### Mission Request Messages (Allowed from All GCS)
These are **NOT** blocked, so backup GCS can still view missions:
- `MISSION_REQUEST_LIST` - Request mission count
- `MISSION_REQUEST` - Request specific waypoint
- `MISSION_REQUEST_INT` - Request waypoint (INT format)

## 🎯 Typical Mission Workflow

### Scenario: Active GCS uploads mission

```
1. Active GCS (255) starts mission upload
   → MISSION_COUNT sent
   → ✓ Forwarded to vehicle
   
2. Vehicle requests waypoints
   → MISSION_REQUEST messages
   
3. Active GCS sends waypoints
   → MISSION_ITEM messages
   → ✓ All forwarded to vehicle
   
4. Backup GCS (254) sees telemetry
   → MISSION_CURRENT updates
   → MISSION_ITEM_REACHED
   → ✓ All telemetry forwarded to backup
```

### Scenario: Backup GCS tries to modify mission

```
1. Backup GCS (254) tries to upload mission
   → MISSION_COUNT sent
   → ✗ BLOCKED by broker
   → Vehicle never sees it
   
2. Backup GCS can still VIEW mission
   → Send MISSION_REQUEST_LIST
   → ✓ Allowed (read-only request)
   → Receives mission data in telemetry
```

## 🔄 Mission Control Transfer

When control transfers, mission uploads work seamlessly:

```python
# Before transfer
GCS-255 (Active)  → Can upload missions ✓
GCS-254 (Backup)  → Cannot upload missions ✗

# GCS-255 disconnects or relinquishes
# GCS-254 becomes active

# After transfer  
GCS-254 (Active)  → Can upload missions ✓
GCS-255 (Backup)  → Cannot upload missions ✗ (if reconnects)
```

## 📊 What Each GCS Sees During Mission

### **Active Controller**
```
✓ Upload waypoints
✓ Modify waypoints
✓ Clear mission
✓ Start mission
✓ Pause mission
✓ Resume mission
✓ Set current waypoint
✓ Receive all mission telemetry
```

### **Backup GCS**
```
✗ Upload waypoints (blocked)
✗ Modify waypoints (blocked)
✗ Clear mission (blocked)
✓ Request mission download (allowed)
✓ View mission progress via telemetry
✓ See MISSION_CURRENT updates
✓ See MISSION_ITEM_REACHED
✓ Monitor mission completion
```

## 🧪 Testing Mission Support

I can add a specific mission test to the test suite if you'd like. Here's what it would verify:Perfect! **Yes, missions and waypoints work fully.** I've also added a dedicated mission test (Test #6) to verify this.

## 📝 Summary

**The broker supports ALL mission operations:**

| Operation | Active GCS | Backup GCS |
|-----------|------------|------------|
| Upload waypoints | ✅ Allowed | ❌ Blocked |
| Modify waypoints | ✅ Allowed | ❌ Blocked |
| Clear mission | ✅ Allowed | ❌ Blocked |
| Set current waypoint | ✅ Allowed | ❌ Blocked |
| Download mission | ✅ Allowed | ✅ **Allowed** (read-only) |
| View mission telemetry | ✅ Receives | ✅ Receives |
| See mission progress | ✅ Receives | ✅ Receives |

## 🎯 Key Point

**Backup GCS are "read-only observers"** for missions:
- They can **request** and **view** the mission
- They can **monitor** mission progress via telemetry
- They **cannot modify** or upload new missions
- Only when they become active can they upload/modify missions

This ensures mission integrity - no conflicting waypoint uploads from multiple GCS!

## Troubleshooting

### My commands aren't working
- Check if you're the active controller (first to connect, or lowest System ID after previous controller disconnected)
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

- **Heartbeat timeout**: `5 seconds` (hard-coded in mavManager.js)
- **Relinquish command**: `COMMAND_LONG (76)` { command: 45000 }
- **GCS detection**: MAVLink type `6` (minimal.MavType.GCS)
- **Initial Control**: First controller to connect becomes the active controller.
- **Fallback priority**: Backup controller with lowest System ID becomes the active controller
- **Monitoring interval**: Every `2 seconds` (hard-coded in mavManager.js)
