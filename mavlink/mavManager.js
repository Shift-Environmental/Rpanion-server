// Mavlink Manager with GCS Connection Management / Relinquish Control
const events = require('events')
const udp = require('dgram')
const { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkProtocolV2, minimal, common, ardupilotmega, MavLinkProtocolV1 } = require('node-mavlink')
const { PassThrough } = require('stream')

// create a registry of mappings between a message id and a data class
const REGISTRY = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY
}

// Custom MAV_CMD for relinquishing control
const MAV_CMD_RELINQUISH_CONTROL = 45000

class GCSConnection {
  constructor(sysId, compId, address, port) {
    this.sysId = sysId
    this.compId = compId
    this.address = address
    this.port = port
    this.lastHeartbeat = Date.now()
    this.isActive = false
  }

  updateHeartbeat() {
    this.lastHeartbeat = Date.now()
  }

  isAlive(timeout = 5000) {
    return (Date.now() - this.lastHeartbeat) < timeout
  }
}

class mavManager {
  constructor (version, inudpIP, inudpPort, enableDSRequest, gcsHeartbeatTimeout = 5000) {
    this.mav = null
    this.mavmsg = null
    this.version = version

    this.eventEmitter = new events.EventEmitter()

    // GCS Connection Management
    this.gcsConnections = new Map() // Map of sysId -> GCSConnection
    this.activeGCS = null // sysId of active controller
    this.gcsHeartbeatTimeout = gcsHeartbeatTimeout

    // Start GCS heartbeat monitoring
    this.startGCSMonitoring()

    // are we in a system reboot?
    this.isRebooting = false

    // System status
    this.statusNumRxPackets = 0
    this.statusBytesPerSec = { avgBytesSec: 0, bytes: 0, lastTime: Date.now().valueOf() }
    this.statusFWName = ''
    this.statusVehType = ''
    this.fcVersion = ''
    this.timeofLastPacket = 0
    this.statusText = ''
    this.statusArmed = 0
    this.seq = 0

    // the vehicle
    this.targetSystem = null
    this.targetComponent = null

    this.enableDSRequest = enableDSRequest

    // udp input
    this.udpStream = udp.createSocket('udp4')
    this.inudpPort = inudpPort
    this.inudpIP = inudpIP
    this.RinudpPort = null
    this.RinudpIP = null
    this.inStream = new PassThrough()

    this.udpStream.on('message', (msg, rinfo) => {
      // calculate bytes/sec rate (once per 2 sec) and do DS requests
      if ((this.statusBytesPerSec.lastTime + 2000) < Date.now().valueOf()) {
        this.statusBytesPerSec.avgBytesSec = Math.round(1000 * this.statusBytesPerSec.bytes / (Date.now().valueOf() - this.statusBytesPerSec.lastTime))
        this.statusBytesPerSec.bytes = 0
        this.statusBytesPerSec.lastTime = Date.now().valueOf()

        if (this.enableDSRequest && this.targetSystem != null && this.targetComponent != null) {
          this.sendDSRequest()
        }
      } else {
        this.statusBytesPerSec.bytes += msg.length
      }

      // lock onto server port
      if (this.RinudpPort === null || this.RinudpIP === null) {
        this.RinudpPort = rinfo.port
        this.RinudpIP = rinfo.address
        console.log(this.RinudpPort)
        this.eventEmitter.emit('linkready', true)
      }

      this.inStream.write(msg)
    })

    this.udpStream.bind(inudpPort, inudpIP)

    this.mav = this.inStream.pipe(new MavLinkPacketSplitter()).pipe(new MavLinkPacketParser())

    // what to do when we get a message
    this.mav.on('data', packet => {
      const clazz = REGISTRY[packet.header.msgid]
      if (!clazz) {
        // bad message - can't process here any further
        this.eventEmitter.emit('gotMessage', packet, null)
        return
      }
      const data = packet.protocol.data(packet.payload, clazz)

      // Handle GCS heartbeats first
      if (this.isGCS(data.type) && packet.header.msgid === minimal.Heartbeat.MSG_ID) {
        this.handleGCSHeartbeat(packet, data)
        return // Don't process GCS heartbeats further
      }

      // Block ALL messages from non-active GCS (except heartbeats handled above)
      if (this.gcsConnections.has(packet.header.sysid) && !this.isActiveGCS(packet.header.sysid)) {
        // This is from a GCS but not the active one - block it
        console.log(`[MAV-MANAGER] Blocked message (msgId=${packet.header.msgid}) from non-active GCS sysId=${packet.header.sysid}`)
        return
      }

      // set the target system/comp ID if needed
      // ensure it's NOT a GCS, as mavlink-router will sometimes route
      // messages from connected GCS's
      if (this.targetSystem === null && packet.header.msgid === minimal.Heartbeat.MSG_ID && !this.isGCS(data.type)) {
        console.log('Vehicle is S/C: ' + packet.header.sysid + '/' + packet.header.compid)
        this.targetSystem = packet.header.sysid
        this.targetComponent = packet.header.compid

        // send off initial messages
        this.sendVersionRequest()

        // Handle relinquish control command
      } else if (packet.header.msgid === common.CommandLong.MSG_ID && 
                 data.command === MAV_CMD_RELINQUISH_CONTROL) {
        this.handleRelinquishControl(packet, data)
        return

        // Respond to MavLink commands that are targeted to the companion computer
      } else if (data.targetSystem === this.targetSystem &&
        data.targetComponent === minimal.MavComponent.ONBOARD_COMPUTER &&
        packet.header.msgid === common.CommandLong.MSG_ID) {
        console.log('Received CommandLong addressed to onboard computer')

      // Or the attached camera
      } else if (data.targetSystem === this.targetSystem &&
        data.targetComponent === minimal.MavComponent.CAMERA &&
        packet.header.msgid === common.CommandLong.MSG_ID) {
        console.log('Received CommandLong addressed to attached camera')

      } else if (this.targetSystem !== packet.header.sysid || this.targetComponent !== packet.header.compid) {
        // don't use packets from other systems or components in Rpanion-server
        return
      }

      // raise event for external objects
      this.eventEmitter.emit('gotMessage', packet, data)

      this.statusNumRxPackets += 1
      this.timeofLastPacket = (Date.now().valueOf())

      // Process vehicle heartbeats to identify target system/component
      if (packet.header.msgid === minimal.Heartbeat.MSG_ID && !this.isGCS(data.type)) {
        // System status
        this.statusFWName = data.autopilot
        this.statusVehType = data.type

        // arming status
        if ((data.baseMode & 128) !== 0 && this.statusArmed === 0) {
          console.log('Vehicle ARMED')
          this.statusArmed = 1
          this.eventEmitter.emit('armed')
        } else if ((data.baseMode & 128) === 0 && this.statusArmed === 1) {
          console.log('Vehicle DISARMED')
          this.statusArmed = 0
          this.eventEmitter.emit('disarmed')
        }
      } else if (packet.header.msgid === common.StatusText.MSG_ID) {
        // Remove whitespace
        this.statusText += data.text.trim().replace(/[^ -~]+/g, '') + '\n'
      } else if (packet.header.msgid === common.AutopilotVersion.MSG_ID) {
        // decode Ardupilot version
        this.fcVersion = this.decodeFlightSwVersion(data.flightSwVersion)
        console.log(this.fcVersion)
      }
    })
  }

  isGCS(mavType) {
    // Check if the MAV type is a GCS (6)
    return mavType === minimal.MavType.GCS
  }

  handleGCSHeartbeat(packet, data) {
    const sysId = packet.header.sysid
    
    if (!this.gcsConnections.has(sysId)) {
      // New GCS connection
      const gcs = new GCSConnection(sysId, packet.header.compid, this.RinudpIP, this.RinudpPort)
      this.gcsConnections.set(sysId, gcs)
      console.log(`New GCS connected: sysId=${sysId}, compId=${packet.header.compid}`)
      
      // Check if this should be the active controller
      this.updateActiveGCS()
    } else {
      // Update existing GCS heartbeat
      console.log(`Received heartbeat from sysId=${sysId}`)
      this.gcsConnections.get(sysId).updateHeartbeat()
    }
  }

  handleRelinquishControl(packet, data) {
    const sysId = packet.header.sysid
    
    console.log(`[MAV-MANAGER] GCS sysId=${sysId} wants to relinquish control`)
    
    if (sysId === this.activeGCS) {
      // Active controller is relinquishing - remove it and find new active
      this.gcsConnections.delete(sysId)
      this.updateActiveGCS()
      this.sendCommandAck(data.command, 0, packet.header.sysid, packet.header.compid, minimal.MavComponent.ONBOARD_COMPUTER)
    } else {
      // Not the active controller
      console.log(`[MAV-MANAGER] GCS sysId=${sysId} is not active controller, cannot relinquish (denied)`)
      this.sendCommandAck(data.command, 4, packet.header.sysid, packet.header.compid, minimal.MavComponent.ONBOARD_COMPUTER) // MAV_RESULT_DENIED
    }
  }

  updateActiveGCS() {
    // Find the alive GCS with the highest system ID
    let newActive = null
    let highestSysId = -1

    for (const [sysId, gcs] of this.gcsConnections.entries()) {
      if (gcs.isAlive(this.gcsHeartbeatTimeout) && sysId > highestSysId) {
        highestSysId = sysId
        newActive = sysId
      }
    }

    // Update active GCS if it changed
    if (newActive !== this.activeGCS) {
      // Clear old active flag
      if (this.activeGCS && this.gcsConnections.has(this.activeGCS)) {
        this.gcsConnections.get(this.activeGCS).isActive = false
      }

      this.activeGCS = newActive

      if (newActive) {
        this.gcsConnections.get(newActive).isActive = true
        console.log(`[MAV-MANAGER] GCS sysId=${newActive} is now the ACTIVE controller`)
        this.eventEmitter.emit('activeGCSChanged', this.gcsConnections.get(newActive))
      } else {
        this.eventEmitter.emit('noActiveGCS')
      }
      
      // Log current GCS state
      this.logGCSState()
    }
  }

  logGCSState() {
    console.log(`[MAV-MANAGER] --- Current GCS State ---`)
    console.log(`[MAV-MANAGER] Total connected: ${this.gcsConnections.size}`)
    console.log(`[MAV-MANAGER] Active: ${this.activeGCS || 'none'}`)
    const backups = Array.from(this.gcsConnections.keys())
      .filter(id => id !== this.activeGCS)
      .sort((a, b) => b - a)
    if (backups.length > 0) {
      console.log(`[MAV-MANAGER] Backups (priority order): ${backups.join(', ')}`)
    }
    console.log(`[MAV-MANAGER] -----------------------`)
  }

  startGCSMonitoring() {
    // Check GCS heartbeats every 2 seconds
    this.gcsMonitorInterval = setInterval(() => {
      this.checkGCSHeartbeats()
    }, 2000)
  }

  checkGCSHeartbeats() {
    let needsUpdate = false

    // Remove timed out GCS connections
    for (const [sysId, gcs] of this.gcsConnections.entries()) {
      if (!gcs.isAlive(this.gcsHeartbeatTimeout)) {
        const wasActive = (sysId === this.activeGCS)
        console.log(`[MAV-MANAGER] GCS sysId=${sysId} timed out (${wasActive ? 'WAS ACTIVE' : 'was backup'})`)
        this.eventEmitter.emit('gcsTimeout', gcs)
        this.gcsConnections.delete(sysId)
        
        if (wasActive) {
          needsUpdate = true
        }
      }
    }

    // Update active GCS if needed
    if (needsUpdate) {
      console.log(`[MAV-MANAGER] Active controller timed out, promoting next highest priority...`)
      this.updateActiveGCS()
    }
  }

  isActiveGCS(sysId) {
    if (this.activeGCS === null) return true // No GCS management yet
    return sysId === this.activeGCS
  }

  getGCSStatus() {
    const backupGCS = Array.from(this.gcsConnections.values())
      .filter(gcs => gcs.sysId !== this.activeGCS)
      .sort((a, b) => b.sysId - a.sysId) // Sort by system ID descending

    return {
      activeGCS: this.activeGCS,
      activeGCSDetails: this.activeGCS ? this.gcsConnections.get(this.activeGCS) : null,
      backupGCS: backupGCS,
      totalGCS: this.gcsConnections.size
    }
  }

  decodeFlightSwVersion (flightSwVersion) {
    // decode 32 bit flight_sw_version mavlink parameter - corresponds to encoding in ardupilot GCS_MAVLINK::send_autopilot_version
    const fwTypeId = (flightSwVersion >> 0) % 256
    const patch = (flightSwVersion >> 8) % 256
    const minor = (flightSwVersion >> 16) % 256
    const major = (flightSwVersion >> 24) % 256
    let fwStr = ''

    switch (fwTypeId) {
      case 0:
        fwStr = 'dev'
        break
      case 64:
        fwStr = 'alpha'
        break
      case 128:
        fwStr = 'beta'
        break
      case 192:
        fwStr = 'rc'
        break
      case 255:
        fwStr = 'official'
        break
      default:
        fwStr = 'Unknown'
        break
    }
    return `${major}.${minor}.${patch}-${fwStr}`
  }

  close () {
    // close cleanly
    if (this.gcsMonitorInterval) {
      clearInterval(this.gcsMonitorInterval)
    }
    if (this.udpStream) {
      this.udpStream.close()
    }
  }

  restart () {
    // reset remote UDP stream
    this.close()
    this.RinudpPort = null
    this.RinudpIP = null
    this.targetSystem = null
    this.targetComponent = null

    // Clear GCS connections
    this.gcsConnections.clear()
    this.activeGCS = null

    this.udpStream = udp.createSocket('udp4')
    this.statusBytesPerSec = { avgBytesSec: 0, bytes: 0, lastTime: Date.now().valueOf() }

    this.udpStream.on('message', (msg, rinfo) => {
      // lock onto server port
      if (this.RinudpPort === null || this.RinudpIP === null) {
        this.RinudpPort = rinfo.port
        this.RinudpIP = rinfo.address
      } else {
        // calculate bytes/sec rate (once per 2 sec) and do DS requests
        if ((this.statusBytesPerSec.lastTime + 2000) < Date.now().valueOf()) {
          this.statusBytesPerSec.avgBytesSec = Math.round(1000 * this.statusBytesPerSec.bytes / (Date.now().valueOf() - this.statusBytesPerSec.lastTime))
          this.statusBytesPerSec.bytes = 0
          this.statusBytesPerSec.lastTime = Date.now().valueOf()

          if (this.enableDSRequest && this.targetSystem != null && this.targetComponent != null) {
            this.sendDSRequest()
          }
        } else {
          this.statusBytesPerSec.bytes += msg.length
        }
        this.inStream.write(msg)
      }
    })

    this.udpStream.bind(this.inudpPort, this.inudpIP)
    this.startGCSMonitoring()
  }

  sendData (msg, component) {
    // Set the default target component if it wasn't specified
    if (component === null || component === undefined) {
      component = minimal.MavComponent.ONBOARD_COMPUTER
    }

    // msgbuf outgoing data
    if (this.RinudpPort === null || this.RinudpIP === null) {
      return
    }

    let protocol = null
    if (this.version === 2) {
      protocol = new MavLinkProtocolV2(this.targetSystem, component)
    } else {
      protocol = new MavLinkProtocolV1(this.targetSystem, component)
    }

    const buffer = protocol.serialize(msg, this.seq++)
    this.seq &= 255

    this.udpStream.send(buffer, this.RinudpPort, this.RinudpIP, function (error) {
      if (error) {
        this.udpStream.close()
        console.log(error)
      }
    })
  }

  sendHeartbeat (mavType, autopilot, component) {

    // Set defaults if parameters are not provided
    if (mavType === null || mavType === undefined) {
      mavType = minimal.MavType.ONBOARD_CONTROLLER
    }

    if (autopilot === null || autopilot === undefined) {
      autopilot = minimal.MavAutopilot.INVALID
    }

    if (component === null || component === undefined) {
      component = minimal.MavComponent.ONBOARD_COMPUTER
    }

      // create a heartbeat packet
    const heartbeatMessage = new minimal.Heartbeat()

    heartbeatMessage.type = mavType
    heartbeatMessage.autopilot = autopilot
    heartbeatMessage.mavlinkVersion = this.version
    // Set these to zero since we aren't currently using them
    heartbeatMessage.baseMode = 0
    heartbeatMessage.customMode = 0
    heartbeatMessage.systemStatus = 0

    this.sendData(heartbeatMessage, component)
  }

  sendCommandAck (commandReceived, commandResult, senderSysId, senderCompId, targetComponent) {
    // Set defaults if parameters are not provided
    if (commandResult === null || commandResult === undefined) {
      commandResult = 0
    }

    if (senderSysId === null || senderSysId === undefined) {
      senderSysId = 255
    }

    if (senderCompId === null || senderCompId === undefined) {
      senderCompId = minimal.MavComponent.MISSION_PLANNER
    }

    // create a CommandAck packet
    const commandAck = new common.CommandAck()
    commandAck.command = commandReceived
    // result = 0 for "accepted and executed"
    commandAck.result = commandResult
    // resultParam2 is for optional additional result information. Not currently used by rpanion.
    commandAck.resultParam2 = 0
    commandAck.targetSystem = senderSysId
    commandAck.targetComponent = targetComponent

    this.sendData(commandAck, senderCompId)
  }

  sendReboot () {
    // create a reboot packet
    const command = new common.PreflightRebootShutdownCommand(this.targetSystem, this.targetComponent)
    command.confirmation = 1
    command.autopilot = 1
    this.isRebooting = true
    this.sendData(command)
  }

  sendDSRequest () {
    // send datastream request
    const msg = new common.RequestDataStream()
    msg.targetSystem = this.targetSystem
    msg.targetComponent = this.targetComponent
    msg.reqStreamId = common.MavDataStream.ALL
    msg.reqMessageRate = 4
    msg.startStop = 1
    this.sendData(msg)
  }

  sendVersionRequest () {
    // request ArduPilot version
    const command = new common.RequestMessageCommand(this.targetSystem, this.targetComponent)
    command.messageId = common.AutopilotVersion.MSG_ID
    command.confirmation = 1
    this.sendData(command)
  }

  sendRTCMMessage (gpmessage, seq) {
    // create a rtcm message for the flight controller
    let flags = 0
    if (gpmessage.length > 180) {
      flags = 1
    }
    // add in the sequence number
    flags |= (seq & 0x1F) << 3

    if (gpmessage.length > 4 * 180) {
      // can't send this with GPS_RTCM_DATA
      return
    }
    // send data in 180 byte parts
    let buf = Buffer.from(gpmessage)
    const msgset = []
    const maxBytes = 180
    while (buf.length > maxBytes) {
        // slice
        msgset.push(buf.slice(0, maxBytes))
        buf = buf.slice(maxBytes)
    }
    msgset.push(buf)

    for (let i = 0, len = msgset.length; i < len; i++) {
      const msg = new common.GpsRtcmData()
      msg.flags = flags | (i << 1)
      msg.len = msgset[i].length
      msg.data = msgset[i]
      this.sendData(msg)
    }
  }

  autopilotFromID () {
    switch (this.statusFWName) {
      case 0:
        return 'Generic'
      case 3:
        return 'APM'
      case 4:
        return 'OpenPilot'
      case 12:
        return 'PX4'
      default:
        return 'Unknown'
    }
  }

  vehicleFromID () {
    switch (this.statusVehType) {
      case 0:
        return 'Generic'
      case 1:
        return 'Fixed Wing'
      case 2:
        return 'Quadcopter'
      case 4:
        return 'Helicopter'
      case 5:
        return 'Antenna Tracker'
      case 6:
        return 'GCS'
      case 10:
        return 'Ground Rover'
      case 11:
        return 'Boat'
      case 12:
        return 'Submarine'
      case 13:
        return 'Hexacopter'
      case 14:
        return 'Octocopter'
      case 15:
        return 'Tricopter'
      default:
        return 'Unknown'
    }
  }

  conStatusStr () {
    // connection status - connected, not connected, no packets for x sec
    if ((Date.now().valueOf()) - this.timeofLastPacket < 5000) {
      return 'Connected'
    } else if (this.timeofLastPacket > 0) {
      return 'Connection lost for ' + (Date.now().valueOf() - this.timeofLastPacket) / 1000 + ' seconds'
    } else {
      return 'Not connected'
    }
  }

  conStatusInt () {
    // connection status - connected (1), not connected (0), no packets for x sec (-1)
    if ((Date.now().valueOf()) - this.timeofLastPacket < 5000) {
      return 1
    } else if (this.timeofLastPacket > 0) {
      return -1
    } else {
      return 0
    }
  }
}

module.exports = mavManager
