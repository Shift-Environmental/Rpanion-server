// Mavlink Manager with GCS Connection Management / Relinquish Control
// Enhanced with detailed logging for debugging
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
    this.heartbeatCount = 0
  }

  updateHeartbeat() {
    this.lastHeartbeat = Date.now()
    this.heartbeatCount++
  }

  isAlive(timeout = 5000) {
    return (Date.now() - this.lastHeartbeat) < timeout
  }

  getTimeSinceLastHB() {
    return Date.now() - this.lastHeartbeat
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

    console.log(`[INIT] GCS heartbeat timeout set to ${gcsHeartbeatTimeout}ms`)

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

    console.log(`[INIT] Listening on ${inudpIP}:${inudpPort}`)

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
        console.log(`[UDP] Locked onto remote ${this.RinudpIP}:${this.RinudpPort}`)
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

      // Log all messages for debugging (except heartbeats to reduce noise)
      if (packet.header.msgid !== minimal.Heartbeat.MSG_ID) {
        console.log(`[MSG-DEBUG] msgId=${packet.header.msgid} from sysId=${packet.header.sysid}`)
      }

      // Handle GCS heartbeats first
      if (this.isGCS(data.type) && packet.header.msgid === minimal.Heartbeat.MSG_ID) {
        this.handleGCSHeartbeat(packet, data)
        return // Don't process GCS heartbeats further
      }

      // Determine if this message is from a known GCS
      const isFromKnownGCS = this.gcsConnections.has(packet.header.sysid)
      
      // Block ALL messages from non-active GCS (except heartbeats handled above)
      if (isFromKnownGCS && !this.isActiveGCS(packet.header.sysid)) {
        // This is from a GCS but not the active one - block it
        console.log(`[MESSAGE-BLOCK] 🚫 Blocked msgId=${packet.header.msgid} from non-active GCS sysId=${packet.header.sysid}`)
        return
      }
      
      // For messages from active GCS, add detailed logging for control messages
      if (isFromKnownGCS && this.isActiveGCS(packet.header.sysid)) {
        if (packet.header.msgid === common.ManualControl?.MSG_ID) {
          console.log(`[MANUAL-CONTROL] ✓ From active GCS sysId=${packet.header.sysid}`)
          console.log(`                 └─ x=${data.x}, y=${data.y}, z=${data.z}, r=${data.r}`)
        }
      }

      // Handle relinquish control command (works with or without flight controller)
      if (packet.header.msgid === common.CommandLong.MSG_ID) {
        console.log(`[COMMAND-LONG] Received command=${data.command} from sysId=${packet.header.sysid}`)
        console.log(`               Target: ${data.targetSystem}/${data.targetComponent}`)
        
        if (data.command === MAV_CMD_RELINQUISH_CONTROL) {
          this.handleRelinquishControl(packet, data)
          return
        }
      }

      // set the target system/comp ID if needed
      // ensure it's NOT a GCS, as mavlink-router will sometimes route
      // messages from connected GCS's
      if (this.targetSystem === null && packet.header.msgid === minimal.Heartbeat.MSG_ID && !this.isGCS(data.type)) {
        console.log(`[VEHICLE-DETECTED] 🚁 Vehicle S/C: ${packet.header.sysid}/${packet.header.compid}`)
        this.targetSystem = packet.header.sysid
        this.targetComponent = packet.header.compid

        // send off initial messages
        this.sendVersionRequest()

        // Respond to MavLink commands that are targeted to the companion computer
      } else if (data.targetSystem === this.targetSystem &&
        data.targetComponent === minimal.MavComponent.ONBOARD_COMPUTER &&
        packet.header.msgid === common.CommandLong.MSG_ID) {
        console.log('[COMMAND] Received CommandLong addressed to onboard computer')

      // Or the attached camera
      } else if (data.targetSystem === this.targetSystem &&
        data.targetComponent === minimal.MavComponent.CAMERA &&
        packet.header.msgid === common.CommandLong.MSG_ID) {
        console.log('[COMMAND] Received CommandLong addressed to attached camera')

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
          console.log('[VEHICLE] Vehicle ARMED')
          this.statusArmed = 1
          this.eventEmitter.emit('armed')
        } else if ((data.baseMode & 128) === 0 && this.statusArmed === 1) {
          console.log('[VEHICLE] Vehicle DISARMED')
          this.statusArmed = 0
          this.eventEmitter.emit('disarmed')
        }
      } else if (packet.header.msgid === common.StatusText.MSG_ID) {
        // Remove whitespace
        this.statusText += data.text.trim().replace(/[^ -~]+/g, '') + '\n'
      } else if (packet.header.msgid === common.AutopilotVersion?.MSG_ID) {
        // decode Ardupilot version
        this.fcVersion = this.decodeFlightSwVersion(data.flightSwVersion)
        console.log(`[VEHICLE] Flight controller version: ${this.fcVersion}`)
      }
    })
  }

  isGCS(mavType) {
    // Check if the MAV type is a GCS (6)
    return mavType === minimal.MavType.GCS
  }

  handleGCSHeartbeat(packet, data) {
    const sysId = packet.header.sysid
    const timestamp = new Date().toISOString().split('T')[1].slice(0, 12) // HH:MM:SS.mmm
    
    if (!this.gcsConnections.has(sysId)) {
      // New GCS connection
      const gcs = new GCSConnection(sysId, packet.header.compid, this.RinudpIP, this.RinudpPort)
      this.gcsConnections.set(sysId, gcs)
      console.log(`[${timestamp}] [GCS-CONNECT] 🟢 NEW GCS connected`)
      console.log(`                    └─ sysId=${sysId}, compId=${packet.header.compid}`)
      console.log(`                    └─ from ${this.RinudpIP}:${this.RinudpPort}`)
      
      // Only become active if there's no active controller
      if (this.activeGCS === null) {
        this.activeGCS = sysId
        gcs.isActive = true
        console.log(`                    └─ ⭐ First GCS becomes ACTIVE`)
        this.logGCSState()
        this.eventEmitter.emit('activeGCSChanged', gcs)
      } else {
        console.log(`                    └─ 📋 Joining as BACKUP (active: ${this.activeGCS})`)
        this.logGCSState()
      }
    } else {
      // Update existing GCS heartbeat
      const gcs = this.gcsConnections.get(sysId)
      const timeSinceLastHB = Date.now() - gcs.lastHeartbeat
      gcs.updateHeartbeat()
      console.log(`[${timestamp}] [GCS-HEARTBEAT] 💓 sysId=${sysId} (${gcs.isActive ? 'ACTIVE' : 'BACKUP'}) gap:${timeSinceLastHB}ms count:${gcs.heartbeatCount}`)
    }
  }

  handleRelinquishControl(packet, data) {
    const sysId = packet.header.sysid
    const timestamp = new Date().toISOString().split('T')[1].slice(0, 12)
    
    console.log(`[${timestamp}] [RELINQUISH] ⚡ Request from sysId=${sysId}`)
    
    if (sysId === this.activeGCS) {
      console.log(`                    └─ ✓ Active controller relinquishing`)
      // Active controller is relinquishing - remove it and find new active
      this.gcsConnections.delete(sysId)
      this.updateActiveGCS()
      this.sendCommandAck(data.command, 0, packet.header.sysid, packet.header.compid, minimal.MavComponent.ONBOARD_COMPUTER)
      console.log(`                    └─ ✓ ACK sent (ACCEPTED)`)
    } else {
      console.log(`                    └─ ✗ Not active controller (current active: ${this.activeGCS})`)
      this.sendCommandAck(data.command, 4, packet.header.sysid, packet.header.compid, minimal.MavComponent.ONBOARD_COMPUTER) // MAV_RESULT_DENIED
      console.log(`                    └─ ✗ ACK sent (DENIED)`)
    }
  }

  updateActiveGCS() {
    const timestamp = new Date().toISOString().split('T')[1].slice(0, 12)
    
    // Find the alive GCS with the HIGHEST system ID (highest priority for failover)
    let newActive = null
    let highestSysId = -1

    console.log(`[${timestamp}] [GCS-UPDATE] 🔄 Finding backup controller...`)
    console.log(`                    └─ Current active: ${this.activeGCS || 'none'}`)
    console.log(`                    └─ Total GCS: ${this.gcsConnections.size}`)

    for (const [sysId, gcs] of this.gcsConnections.entries()) {
      const alive = gcs.isAlive(this.gcsHeartbeatTimeout)
      const timeSince = gcs.getTimeSinceLastHB()
      console.log(`                    └─ sysId=${sysId}: alive=${alive}, lastHB=${timeSince}ms ago`)
      
      if (alive && sysId > highestSysId) {
        highestSysId = sysId
        newActive = sysId
      }
    }

    // Update active GCS if it changed
    if (newActive !== this.activeGCS) {
      // Clear old active flag
      if (this.activeGCS && this.gcsConnections.has(this.activeGCS)) {
        this.gcsConnections.get(this.activeGCS).isActive = false
        console.log(`                    └─ 🔻 Deactivated sysId=${this.activeGCS}`)
      }

      this.activeGCS = newActive

      if (newActive) {
        this.gcsConnections.get(newActive).isActive = true
        console.log(`                    └─ ⭐ PROMOTED sysId=${newActive} to ACTIVE (highest priority backup)`)
        this.eventEmitter.emit('activeGCSChanged', this.gcsConnections.get(newActive))
      } else {
        console.log(`                    └─ ⚠️  NO ACTIVE GCS`)
        this.eventEmitter.emit('noActiveGCS')
      }
      
      // Log current GCS state
      this.logGCSState()
    } else {
      console.log(`                    └─ No change (active remains: ${this.activeGCS || 'none'})`)
    }
  }

  logGCSState() {
    const timestamp = new Date().toISOString().split('T')[1].slice(0, 12)
    console.log(`[${timestamp}] [GCS-STATE] ═══════════════════════════`)
    console.log(`                    │ Total connected: ${this.gcsConnections.size}`)
    console.log(`                    │ Active: ${this.activeGCS || 'none'}`)
    const backups = Array.from(this.gcsConnections.keys())
      .filter(id => id !== this.activeGCS)
      .sort((a, b) => b - a) // Sort descending - higher IDs have higher priority
    if (backups.length > 0) {
      console.log(`                    │ Backups (priority): ${backups.join(', ')}`)
    }
    
    // Show details of each GCS
    for (const [sysId, gcs] of this.gcsConnections.entries()) {
      const status = gcs.isActive ? 'ACTIVE' : 'BACKUP'
      const timeSince = gcs.getTimeSinceLastHB()
      console.log(`                    │ • sysId=${sysId}: ${status}, lastHB=${timeSince}ms ago, count=${gcs.heartbeatCount}`)
    }
    console.log(`                    ═══════════════════════════`)
  }

  startGCSMonitoring() {
    console.log('[INIT] Starting GCS heartbeat monitoring (every 2s)')
    // Check GCS heartbeats every 2 seconds
    this.gcsMonitorInterval = setInterval(() => {
      this.checkGCSHeartbeats()
    }, 2000)
  }

  checkGCSHeartbeats() {
    const timestamp = new Date().toISOString().split('T')[1].slice(0, 12)
    let needsUpdate = false
    let timeoutDetected = false

    // Remove timed out GCS connections
    for (const [sysId, gcs] of this.gcsConnections.entries()) {
      if (!gcs.isAlive(this.gcsHeartbeatTimeout)) {
        const wasActive = (sysId === this.activeGCS)
        const timeSince = gcs.getTimeSinceLastHB()
        
        if (!timeoutDetected) {
          console.log(`[${timestamp}] [GCS-TIMEOUT] 🔴 Timeout check...`)
          timeoutDetected = true
        }
        
        console.log(`                     └─ sysId=${sysId} TIMED OUT (${wasActive ? 'WAS ACTIVE' : 'was backup'})`)
        console.log(`                     └─ Last heartbeat: ${timeSince}ms ago (threshold: ${this.gcsHeartbeatTimeout}ms)`)
        
        this.eventEmitter.emit('gcsTimeout', gcs)
        this.gcsConnections.delete(sysId)
        
        if (wasActive) {
          needsUpdate = true
        }
      }
    }

    // Update active GCS if needed
    if (needsUpdate) {
      console.log(`                     └─ ⚠️  Active controller lost, promoting backup...`)
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
      .sort((a, b) => b.sysId - a.sysId) // Sort by system ID descending - higher = higher priority

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
    console.log('[SHUTDOWN] Closing mavManager...')
    if (this.gcsMonitorInterval) {
      clearInterval(this.gcsMonitorInterval)
    }
    if (this.udpStream) {
      this.udpStream.close()
    }
  }

  restart () {
    console.log('[RESTART] Restarting mavManager...')
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
    console.log('[RESTART] Complete')
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
    try {
      // Check if AutopilotVersion exists before using it
      if (!common.AutopilotVersion) {
        console.log('[VERSION-REQUEST] AutopilotVersion not available in node-mavlink, skipping version request')
        return
      }
      
      const command = new common.RequestMessageCommand(this.targetSystem, this.targetComponent)
      command.messageId = common.AutopilotVersion.MSG_ID
      command.confirmation = 1
      this.sendData(command)
    } catch (err) {
      console.log('[VERSION-REQUEST] Error sending version request:', err.message)
    }
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
