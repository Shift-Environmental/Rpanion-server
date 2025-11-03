// GCS Relinquish Control Test
// Tests control handoff between Field Control System and Handheld Controller
const dgram = require('dgram')
const { MavLinkProtocolV2, minimal, common } = require('node-mavlink')

const SERVER_IP = '127.0.0.1'
const SERVER_PORT = 14540
const MAV_CMD_RELINQUISH_CONTROL = 45000

class GCSController {
  constructor(sysId, compId, name) {
    this.sysId = sysId
    this.compId = compId
    this.name = name
    this.socket = dgram.createSocket('udp4')
    this.heartbeatInterval = null
    this.seq = 0
    this.isRunning = false
  }

  connect() {
    return new Promise((resolve) => {
      this.socket.bind(() => {
        console.log(`[${this.name}] Connected on port ${this.socket.address().port}`)
        this.isRunning = true
        this.startHeartbeat()
        resolve()
      })

      this.socket.on('message', (msg) => {
        // Listen for server responses (could parse MAVLink here if needed)
      })
    })
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.isRunning) {
        this.sendHeartbeat()
      }
    }, 1000) // 1 Hz
  }

  sendHeartbeat() {
    const heartbeat = new minimal.Heartbeat()
    heartbeat.type = minimal.MavType.GCS
    heartbeat.autopilot = minimal.MavAutopilot.INVALID
    heartbeat.baseMode = 0
    heartbeat.customMode = 0
    heartbeat.systemStatus = minimal.MavState.ACTIVE
    heartbeat.mavlinkVersion = 3

    this.sendMessage(heartbeat)
  }

  relinquishControl() {
    console.log(`[${this.name}] ⚡ Sending RELINQUISH_CONTROL command`)
    
    const command = new common.CommandLong()
    command.targetSystem = 1
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
    console.log(`[${this.name}] 🔴 Stopping heartbeat (simulating connection loss)`)
    this.isRunning = false
  }

  resumeHeartbeat() {
    console.log(`[${this.name}] 🟢 Resuming heartbeat`)
    this.isRunning = true
  }

  disconnect() {
    console.log(`[${this.name}] Disconnecting`)
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval)
    }
    this.socket.close()
  }
}

// Test Runner
class RelinquishTest {
  constructor() {
    this.fieldControl = new GCSController(255, 190, 'Field-Control-System')
    this.handheld = new GCSController(200, 190, 'Handheld-Controller')
  }

  async run() {
    console.log('\n' + '='.repeat(60))
    console.log('GCS RELINQUISH CONTROL TEST')
    console.log('='.repeat(60) + '\n')

    try {
      await this.test1_InitialConnection()
      await this.test2_VoluntaryRelinquish()
      await this.test3_ConnectionLossFailover()
      await this.test4_ReconnectAfterLoss()
      await this.test4b_NonActiveRelinquishAttempt()
      await this.test5_RelinquishWithOneController()

      console.log('\n' + '='.repeat(60))
      console.log('ALL TESTS COMPLETED')
      console.log('='.repeat(60) + '\n')
    } catch (error) {
      console.error('Test error:', error)
    } finally {
      this.cleanup()
    }
  }

  async test1_InitialConnection() {
    console.log('\n📌 TEST 1: Initial Connection')
    console.log('─'.repeat(60))
    console.log('Connecting Field Control System first...\n')

    await this.fieldControl.connect()
    await this.sleep(2000)

    console.log('\nConnecting Handheld Controller...\n')
    await this.handheld.connect()
    await this.sleep(2000)

    console.log('✓ Expected: Field Control System is ACTIVE (first to connect)')
    console.log('✓ Expected: Handheld Controller is BACKUP')
    console.log('✓ Expected: Active controller does NOT change when backup connects\n')
    await this.sleep(2000)
  }

  async test2_VoluntaryRelinquish() {
    console.log('\n📌 TEST 2: Voluntary Relinquish Control')
    console.log('─'.repeat(60))
    console.log('Field Control System (ACTIVE) voluntarily relinquishes control...\n')

    // Send relinquish command
    this.fieldControl.relinquishControl()
    await this.sleep(1000)

    // Stop heartbeats to simulate disconnection after relinquish
    this.fieldControl.stopHeartbeat()
    console.log('[Field-Control-System] 🔴 Stopping heartbeat after relinquish\n')
    
    await this.sleep(3000)

    console.log('✓ Expected: Field Control relinquishes and stops heartbeats')
    console.log('✓ Expected: Handheld Controller becomes ACTIVE (only remaining controller)\n')
    
    console.log('Field Control reconnecting...\n')
    this.fieldControl.resumeHeartbeat()
    await this.sleep(3000)
    
    console.log('✓ Expected: Field Control reconnects as BACKUP')
    console.log('✓ Expected: Handheld remains ACTIVE (first-connected-stays-active)\n')
    await this.sleep(2000)
  }

  async test3_ConnectionLossFailover() {
    console.log('\n📌 TEST 3: Connection Loss Failover')
    console.log('─'.repeat(60))
    console.log('Simulating Handheld Controller connection loss...\n')

    this.handheld.stopHeartbeat()
    console.log('Waiting for heartbeat timeout (7 seconds)...')
    await this.sleep(7000)

    console.log('\n✓ Expected: Handheld Controller times out')
    console.log('✓ Expected: Field Control System becomes ACTIVE')
    console.log('            (sysId 255 > 200, Field Control has highest priority)\n')
    await this.sleep(2000)
  }

  async test4_ReconnectAfterLoss() {
    console.log('\n📌 TEST 4: Reconnect After Connection Loss')
    console.log('─'.repeat(60))
    console.log('Handheld Controller reconnecting...\n')

    this.handheld.resumeHeartbeat()
    await this.sleep(3000)

    console.log('✓ Expected: Handheld Controller reconnects as BACKUP')
    console.log('✓ Expected: Field Control System remains ACTIVE\n')
    await this.sleep(2000)
  }

  async test4b_NonActiveRelinquishAttempt() {
    console.log('\n📌 TEST 4b: Non-Active Controller Relinquish Attempt')
    console.log('─'.repeat(60))
    console.log('Handheld (BACKUP) attempting to relinquish...\n')

    this.handheld.relinquishControl()
    await this.sleep(2000)

    console.log('✓ Expected: Relinquish command is DENIED (not active controller)')
    console.log('✓ Expected: Field Control remains ACTIVE')
    console.log('✓ Expected: Handheld remains BACKUP (no change)\n')
    await this.sleep(2000)
  }

  async test5_RelinquishWithOneController() {
    console.log('\n📌 TEST 5: Relinquish With Only One Controller')
    console.log('─'.repeat(60))
    console.log('Stopping Handheld Controller to leave only Field Control active...\n')

    this.handheld.stopHeartbeat()
    await this.sleep(7000)

    console.log('✓ Handheld timed out, only Field Control remains\n')
    console.log('Field Control (only remaining controller) attempting to relinquish...\n')
    
    this.fieldControl.relinquishControl()
    await this.sleep(3000)

    console.log('✓ Expected: Relinquish command is sent')
    console.log('✓ Expected: Field Control is removed from connections')
    console.log('✓ Expected: No active GCS (no controllers left)')
    console.log('✓ Note: In production, Field Control would likely reconnect')
    console.log('        and become active again (only controller available)\n')
  }

  async sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms))
  }

  cleanup() {
    console.log('\n🧹 Cleaning up...')
    this.fieldControl.disconnect()
    this.handheld.disconnect()
    setTimeout(() => process.exit(0), 500)
  }
}

// Run the test
const test = new RelinquishTest()
test.run()

// Handle interrupt
process.on('SIGINT', () => {
  console.log('\n\n⚠️  Test interrupted by user')
  process.exit(0)
})