// GCS Connection Test Script
// Tests multiple GCS connections with various scenarios
const dgram = require('dgram')
const { MavLinkProtocolV2, minimal, common } = require('node-mavlink')

const SERVER_IP = '172.22.230.179'
const SERVER_PORT = 14550

// Custom MAV_CMD for relinquishing control
const MAV_CMD_RELINQUISH_CONTROL = 42700

class TestGCS {
  constructor(sysId, compId, name) {
    this.sysId = sysId
    this.compId = compId
    this.name = name
    this.socket = dgram.createSocket('udp4')
    this.isActive = false
    this.shouldSendHeartbeat = true
    this.heartbeatInterval = null
    this.seq = 0
    
    console.log(`[${this.name}] Created (sysId=${sysId}, compId=${compId})`)
  }

  start() {
    // Bind to a random port
    this.socket.bind(() => {
      console.log(`[${this.name}] Started on port ${this.socket.address().port}`)
      this.startHeartbeat()
    })

    // Listen for responses
    this.socket.on('message', (msg, rinfo) => {
      // Just log that we received something (could parse responses here)
      // console.log(`[${this.name}] Received ${msg.length} bytes`)
    })
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.shouldSendHeartbeat) {
        this.sendHeartbeat()
      }
    }, 1000) // 1 Hz
  }

  sendHeartbeat() {
    const heartbeat = new minimal.Heartbeat()
    heartbeat.type = minimal.MavType.GCS // Type 6 - GCS
    heartbeat.autopilot = minimal.MavAutopilot.INVALID
    heartbeat.baseMode = 0
    heartbeat.customMode = 0
    heartbeat.systemStatus = minimal.MavState.ACTIVE
    heartbeat.mavlinkVersion = 3

    this.sendMessage(heartbeat)
  }

  sendRelinquish() {
    console.log(`[${this.name}] Sending RELINQUISH_CONTROL command`)
    
    // Create a COMMAND_LONG message
    const command = new common.CommandLong()
    command.targetSystem = 1 // Assuming vehicle is system 1
    command.targetComponent = minimal.MavComponent.ONBOARD_COMPUTER
    command.command = MAV_CMD_RELINQUISH_CONTROL
    command.confirmation = 0
    command.param1 = 0
    command.param2 = 0
    command.param3 = 0
    command.param4 = 0
    command.param5 = 0
    command.param6 = 0
    command.param7 = 0

    this.sendMessage(command)
  }

  sendMessage(message) {
    const protocol = new MavLinkProtocolV2(this.sysId, this.compId)
    const buffer = protocol.serialize(message, this.seq++)
    this.seq &= 255

    this.socket.send(buffer, SERVER_PORT, SERVER_IP, (err) => {
      if (err) {
        console.error(`[${this.name}] Send error:`, err)
      }
    })
  }

  stopHeartbeat() {
    console.log(`[${this.name}] Stopped sending heartbeats`)
    this.shouldSendHeartbeat = false
  }

  resumeHeartbeat() {
    console.log(`[${this.name}] Resumed sending heartbeats`)
    this.shouldSendHeartbeat = true
  }

  disconnect() {
    console.log(`[${this.name}] Disconnecting`)
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval)
    }
    this.socket.close()
  }
}

// Test orchestrator
class GCSTestOrchestrator {
  constructor() {
    this.gcsInstances = []
    this.testPhase = 0
  }

  async runTests() {
    console.log('\n========================================')
    console.log('GCS CONNECTION TEST SUITE')
    console.log('========================================\n')

    await this.phase1_BasicConnections()
    await this.sleep(8000)

    await this.phase2_DuplicateSystemIDs()
    await this.sleep(8000)

    await this.phase3_HeartbeatTimeout()
    await this.sleep(8000)

    await this.phase4_ActiveRelinquish()
    await this.sleep(8000)

    await this.phase5_MultipleBackups()
    await this.sleep(8000)

    await this.phase6_ReconnectScenarios()
    await this.sleep(5000)

    console.log('\n========================================')
    console.log('TEST SUITE COMPLETED')
    console.log('========================================\n')
    
    this.cleanup()
  }

  async phase1_BasicConnections() {
    console.log('\n--- PHASE 1: Basic Connections ---')
    console.log('Testing: First connected becomes active\n')

    const gcs1 = new TestGCS(100, 190, 'GCS-100-A')
    const gcs2 = new TestGCS(255, 190, 'GCS-255-A')
    const gcs3 = new TestGCS(200, 190, 'GCS-200-A')

    gcs1.start()
    await this.sleep(2000)
    
    gcs2.start()
    await this.sleep(2000)
    
    gcs3.start()
    
    console.log('\nExpected: GCS-100-A should be active (first connected)')
    console.log('         GCS-255-A and GCS-200-A should be backups')

    this.gcsInstances.push(gcs1, gcs2, gcs3)
  }

  async phase2_DuplicateSystemIDs() {
    console.log('\n--- PHASE 2: Duplicate System IDs ---')
    console.log('Testing: Second GCS with same sysId should be rejected\n')

    const gcs4 = new TestGCS(255, 191, 'GCS-255-B-DUPLICATE')
    gcs4.start()
    
    await this.sleep(2000)
    
    console.log('\nExpected: GCS-255-B-DUPLICATE should be rejected (sysId 255 already exists)')

    this.gcsInstances.push(gcs4)
  }

  async phase3_HeartbeatTimeout() {
    console.log('\n--- PHASE 3: Heartbeat Timeout ---')
    console.log('Testing: Active controller timeout causes failover\n')

    // Stop the active controller (GCS-100-A)
    const activeGCS = this.gcsInstances[0]
    activeGCS.stopHeartbeat()
    
    console.log('Waiting for timeout (5+ seconds)...')
    await this.sleep(7000)
    
    console.log('\nExpected: GCS-100-A should timeout')
    console.log('         GCS-255-A should become active (highest sysId)')
  }

  async phase4_ActiveRelinquish() {
    console.log('\n--- PHASE 4: Active Relinquish ---')
    console.log('Testing: Active controller voluntarily relinquishes\n')

    // GCS-255-A should now be active, make it relinquish
    const gcs255 = this.gcsInstances[1]
    gcs255.sendRelinquish()
    
    await this.sleep(3000)
    
    console.log('\nExpected: GCS-255-A should relinquish and reconnect as backup')
    console.log('         GCS-200-A should become active (next highest sysId)')
  }

  async phase5_MultipleBackups() {
    console.log('\n--- PHASE 5: Multiple Backups Failover ---')
    console.log('Testing: Sequential failover through backup controllers\n')

    // Add more GCS instances
    const gcs5 = new TestGCS(150, 190, 'GCS-150-A')
    const gcs6 = new TestGCS(50, 190, 'GCS-50-A')
    
    gcs5.start()
    await this.sleep(1000)
    gcs6.start()
    await this.sleep(2000)

    this.gcsInstances.push(gcs5, gcs6)

    console.log('\nCurrent backups: GCS-255-A, GCS-150-A, GCS-50-A')
    console.log('Active: GCS-200-A')
    
    // Stop active controller
    const gcs200 = this.gcsInstances[2]
    gcs200.stopHeartbeat()
    
    console.log('\nStopping GCS-200-A heartbeat...')
    await this.sleep(7000)
    
    console.log('\nExpected: GCS-255-A should become active (highest sysId among backups)')
  }

  async phase6_ReconnectScenarios() {
    console.log('\n--- PHASE 6: Reconnect Scenarios ---')
    console.log('Testing: Timed out GCS reconnecting\n')

    // Resume GCS-100-A heartbeat (was stopped in phase 3)
    const gcs100 = this.gcsInstances[0]
    gcs100.resumeHeartbeat()
    
    await this.sleep(3000)
    
    console.log('\nExpected: GCS-100-A reconnects as backup (not taking control)')
    console.log('         GCS-255-A should remain active')

    await this.sleep(2000)

    // Test relinquish with only one GCS
    console.log('\n--- Testing: Single GCS relinquish ---')
    console.log('Stopping all GCS except GCS-255-A...\n')
    
    for (let i = 0; i < this.gcsInstances.length; i++) {
      if (i !== 1) { // Keep GCS-255-A (index 1)
        this.gcsInstances[i].stopHeartbeat()
      }
    }

    await this.sleep(7000)

    console.log('\nOnly GCS-255-A should remain')
    const gcs255 = this.gcsInstances[1]
    gcs255.sendRelinquish()
    
    await this.sleep(3000)
    
    console.log('\nExpected: GCS-255-A relinquishes and immediately regains control')
  }

  async sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms))
  }

  cleanup() {
    console.log('\nCleaning up all GCS connections...')
    for (const gcs of this.gcsInstances) {
      gcs.disconnect()
    }
    process.exit(0)
  }
}

// Run the tests
const orchestrator = new GCSTestOrchestrator()
orchestrator.runTests().catch(err => {
  console.error('Test error:', err)
  process.exit(1)
})

// Handle cleanup on interrupt
process.on('SIGINT', () => {
  console.log('\nTest interrupted')
  orchestrator.cleanup()
})