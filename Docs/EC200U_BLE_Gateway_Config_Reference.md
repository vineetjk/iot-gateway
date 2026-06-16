# EC200U BLE Gateway Configuration Reference
## Module: EC200UCNAA-N05-SGNSA | Firmware: EC200UCNAAR03A03M08
## Use Case: IoT Gateway Parameter Configuration via BLE (GATT Server)

---

## Overview

The EC200U exposes a BLE GATT Server. A mobile app (nRF Connect or custom app) connects
to the gateway and reads/writes characteristics to configure parameters such as:
- MQTT broker IP, port, credentials
- Modbus slave IDs and baud rate
- Device ID, location tag
- Polling intervals
- Network APN settings

**Architecture:**
```
Mobile App (nRF Connect / Custom App)
        ↕ BLE GATT (Write/Read/Notify)
EC200U BLE GATT Server
        ↕ UART AT Commands
STM32F103 Application Firmware
        ↕ Internal flash / NVM
Gateway Configuration Parameters
```

---

## 1. Initialization Sequence (Run Once on Boot)

Always power cycle BLE before starting to avoid stale state errors (CME ERROR 53).

```
AT+QBTPWR=0                          // Turn off BLE (clean state)
AT+QBTPWR=1                          // Turn on BLE GATT Server mode
AT+QBTNAME=0,"<device_name>"         // Set BLE device name (max 29 bytes, UTF8)
```

**Parameter:**
- `AT+QBTPWR=<enable>` — ORed values:
  - `0` = Turn off all BT
  - `1` = BLE GATT Server
  - `2` = BLE GATT Client
  - `3` = Both Server + Client
  - `4` = SPP (Classic BT)

---

## 2. Advertising Setup (Makes Device Visible to Mobile)

### 2.1 Set Advertising Parameters
```
AT+QBTGATADV=1,128,160,0,0,7,0
```
Parameters: `<op>,<min_interval>,<max_interval>,<adv_type>,<addr_type>,<channel_map>,<filter_policy>`
- `1` = Set parameters
- `128` = Min interval (80ms → 128 × 0.625ms)
- `160` = Max interval (100ms → 160 × 0.625ms)
- `0` = Connectable undirected advertising
- `0` = Public address
- `7` = All channels (37, 38, 39)
- `0` = Accept all scan and connection requests

### 2.2 Set Advertising Data
```
AT+QBTADVDATA=<length>,"<hex_data>"
```
Example (flags only):
```
AT+QBTADVDATA=3,"020106"
```
- `03` = length byte not counted, packet is 3 bytes
- `02` = AD length
- `01` = AD Type: Flags
- `06` = LE General Discoverable + BR/EDR not supported

### 2.3 Set Scan Response Data (Device Name visible in scanner)
```
AT+QBTADVRSPDATA=<length>,"<hex_data>"
```
To encode your device name as hex:
- Each character → ASCII hex byte
- Length byte = (name length + 1 for AD Type byte)
- AD Type for Complete Local Name = `09`

Example for "DeltaGW" (7 chars):
```
AT+QBTADVRSPDATA=9,"080944656C74614757"
```
- `09` = total packet length (8 + 1)
- `08` = AD length (7 chars + 1 for type byte)
- `09` = AD Type: Complete Local Name
- `44656C74614757` = "DeltaGW" in ASCII hex

**ASCII to Hex quick reference:**
```
A=41 B=42 C=43 D=44 E=45 F=46 G=47 H=48 I=49
J=4A K=4B L=4C M=4D N=4E O=4F P=50 Q=51 R=52
S=53 T=54 U=55 V=56 W=57 X=58 Y=59 Z=5A
a=61 b=62 c=63 d=64 e=65 f=66 g=67 h=68 i=69
j=6A k=6B l=6C m=6D n=6E o=6F p=70 q=71 r=72
s=73 t=74 u=75 v=76 w=77 x=78 y=79 z=7A
0=30 1=31 2=32 3=33 4=34 5=35 6=36 7=37 8=38 9=39
-(hyphen)=2D _(underscore)=5F .(dot)=2E :(colon)=3A
```

---

## 3. GATT Service & Characteristic Setup

Design one service with multiple characteristics — one per config parameter group.

### 3.1 Add Primary Service
```
AT+QBTGATSS=<servID>,<UUID_type>[,<UUID_128>][,<UUID_16>],<primary>
```
Example (16-bit UUID):
```
AT+QBTGATSS=0,1,6159,1
```
- `0` = Service ID
- `1` = 16-bit UUID type
- `6159` = Service UUID (custom, choose any unused value)
- `1` = Primary service

### 3.2 Add Characteristic
```
AT+QBTGATSC=<servID>,<charaID>,<properties>,<UUID_type>[,<UUID_128>][,<UUID_16>]
```
Properties (ORed bits):
- `Bit1` = Read (value 2)
- `Bit2` = Write without response (value 4)
- `Bit3` = Write (value 8)
- `Bit4` = Notify (value 16)
- `Bit5` = Indicate (value 32)
- `58` = Read + Write + Notify + Indicate (2+8+16+32)
- `10` = Read + Notify (2+8)
- `3`  = Read + Write (2+8) — note: doc uses 3 for permission, 58 for properties

Example (Read + Write + Notify):
```
AT+QBTGATSC=0,0,58,1,10777
```
- `0` = Service ID
- `0` = Characteristic ID
- `58` = Properties: Read+Write+Notify+Indicate
- `1` = 16-bit UUID type
- `10777` = Characteristic UUID

### 3.3 Configure Characteristic Value
```
AT+QBTGATSCV=<servID>,<charaID>,<permission>,<UUID_type>[,<UUID_128>][,<UUID_16>],<value_length>,<value>
```
Permission (ORed bits):
- `Bit0` = Read only (1)
- `Bit1` = Write only (2)
- `3` = Read + Write

Example:
```
AT+QBTGATSCV=0,0,3,1,10777,244,"0000"
```
- `244` = Max value buffer size in bytes (max 512)
- `"0000"` = Initial value

### 3.4 Add Characteristic Descriptor (optional, needed for Notify)
```
AT+QBTGATSCD=<servID>,<charaID>,<permission>,<UUID_type>[,<UUID_128>][,<UUID_16>],<value_length>,<value>
```
Example:
```
AT+QBTGATSCD=0,0,3,1,10498,2,"0300"
```
- `10498` = UUID 0x2902 (Client Characteristic Configuration Descriptor — standard for notify)
- `2` = 2 bytes
- `"0300"` = Default value

### 3.5 Finish Service Registration
```
AT+QBTGATSSC=<type>[,<op>]
```
- `AT+QBTGATSSC=1,1` = Finish adding services, keep default GAP/GATT services
- `AT+QBTGATSSC=0`   = Clear all services

```
AT+QBTGATSSC=1,1
```

---

## 4. Start / Stop Advertising
```
AT+QBTADV=<enable>
```
- `AT+QBTADV=1` = Start advertising (device becomes visible)
- `AT+QBTADV=0` = Stop advertising (device hidden)

**Recommended pattern for industrial use:**
- Start advertising only on button press or power-on window
- Auto-stop after 60 seconds or on successful connection
- Never leave always-on in production

---

## 5. Receiving Data from Mobile App (Phone → Gateway)

When mobile writes to a characteristic, STM32 receives this URC from EC200U UART:

```
+QBTLEVALDATA: <cid>,<phone_mac>,<length>,"<hex_data>"
```

Example — phone writes "HELLO":
```
+QBTLEVALDATA: 0,"3af3f58716f9",5,"48454C4C4F"
```
- `0` = Channel ID
- `"3af3f58716f9"` = Phone MAC address
- `5` = Data length in bytes
- `"48454C4C4F"` = "HELLO" in hex

**In buffer mode** (use `AT+QBTLERCVM=1,<timeout_ms>`):
```
+QBTLEVALDATI: <cid>,<phone_mac>,<length>
```
Then read manually:
```
AT+QBTLEREAD=<cid>,<length>
→ +QBTLEREAD: <cid>,<length>,"<hex_data>"
```

### Set Receiving Mode
```
AT+QBTLERCVM=<type>,<time>
```
- `type=0` = Direct mode (URC fires immediately with data)
- `type=1` = Buffer mode (URC fires after timeout, read manually)
- `time` = timeout in ms (e.g. 2000 = 2 seconds)

---

## 6. Sending Data from Gateway to Mobile (Gateway → Phone)

### 6.1 Send Notification
```
AT+QBTGATSNOD=<op>,<connID>,<att_handle>,<length>,"<hex_data>"
```
- `op=0` = Direct mode
- `op=1` = Transparent transmission mode

Example:
```
AT+QBTGATSNOD=0,0,18,4,"00110011"
```

### 6.2 Send Indication (phone must acknowledge)
```
AT+QBTGATSIND=<op>,<connID>,<att_handle>,<length>,"<hex_data>"
```
Example:
```
AT+QBTGATSIND=0,0,18,4,"11111111"
```

### 6.3 Update Characteristic Value (without sending)
```
AT+QBTGATCHSCV=<servID>,<charaID>,<length>,"<hex_data>"
```
Example:
```
AT+QBTGATCHSCV=0,0,2,"0012"
```

---

## 7. Connection Management

### Check Connection State
```
AT+QBTLESTATE?
→ +QBTLESTATE: <cid>,<connID>,<address>,<conn_state>,<att_handle>
```
- `conn_state=0` = Disconnected
- `conn_state=1` = Connected

### Disconnect a Device
```
AT+QBTGATDISCONN=<connID>
```
Example:
```
AT+QBTGATDISCONN=0
```

### Update Connection Parameters
```
AT+QBTGATCONNP=<connID>,<min_interval>,<max_interval>,<latency>,<timeout>
```
Example (fast connection, 20s timeout):
```
AT+QBTGATCONNP=0,6,6,0,2000
```
- Interval unit = 1.25ms, so 6 = 7.5ms
- Timeout unit = 10ms, so 2000 = 20s

### Exchange MTU
```
AT+QBTLEEXMTU=<connID>,<MTU>
```
- MTU range: 23–247 bytes
- Larger MTU = faster data transfer for config payloads

Example:
```
AT+QBTLEEXMTU=0,220
```

---

## 8. Connection URCs (Unsolicited — STM32 must parse these)

| URC | Meaning |
|-----|---------|
| `+QBTGATSCON: <connID>,"<mac>"` | Phone connected |
| `+QBTGATSDCON: <connID>,"<mac>"` | Phone disconnected |
| `+QBTLESTATE: <cid>,<connID>,"<mac>",<state>,<handle>` | Connection state update |
| `+QBTGATMTU: <connID>,<mtu>` | MTU negotiated |
| `+QBTLEVALDATA: <cid>,"<mac>",<len>,"<data>"` | Data received from phone (direct mode) |
| `+QBTLEVALDATI: <cid>,"<mac>",<len>` | Data received from phone (buffer mode) |
| `+QBTGATDESCDATA: <connID>,<handle>,<len>,"<data>"` | Phone wrote descriptor |
| `+QBTGATRDDATAIND: <connID>,<handle>,<len>,"<data>"` | Phone read characteristic |
| `+QBTGATCONNP: <connID>,<min>,<max>,<lat>,<timeout>` | Connection params updated |

---

## 9. White List (Security — restrict which phones can connect)

### Add phone to whitelist
```
AT+QBTLEADDWHL=<addr_type>,<address>
```
- `addr_type=0` = Public address
- `addr_type=1` = Random address

Example:
```
AT+QBTLEADDWHL=0,"112233da8040"
```

### View whitelist
```
AT+QBTLEWHLINFO?
→ +QBTLEWHLINFO: <addr_type>,<address>
```

### Remove from whitelist
```
AT+QBTLEDELWHL=0          // Remove all
AT+QBTLEDELWHL=1,0,"<mac>"  // Remove specific
```

---

## 10. Recommended GATT Profile for Gateway Config

Design suggestion — one service, multiple characteristics:

| Char ID | UUID  | Properties | Purpose | Max Size |
|---------|-------|------------|---------|----------|
| 0 | 0xAA01 | Read+Write | MQTT Broker IP:Port | 32 bytes |
| 1 | 0xAA02 | Read+Write | MQTT Username+Password | 64 bytes |
| 2 | 0xAA03 | Read+Write | Device ID / Location Tag | 32 bytes |
| 3 | 0xAA04 | Read+Write | Modbus Config (baud, parity, slave IDs) | 16 bytes |
| 4 | 0xAA05 | Read+Write | APN / Network config | 32 bytes |
| 5 | 0xAA06 | Read+Notify | Status / Firmware version (read-only) | 32 bytes |
| 6 | 0xAA07 | Write | Command channel (reboot, reset config) | 8 bytes |

---

## 11. Complete Working Sequence (Copy-Paste Ready)

```
// === INIT ===
AT+QBTPWR=0
AT+QBTPWR=1
AT+QBTNAME=0,"DeltaGW"

// === ADVERTISING ===
AT+QBTGATADV=1,128,160,0,0,7,0
AT+QBTADVDATA=3,"020106"
AT+QBTADVRSPDATA=9,"080944656C74614757"    // "DeltaGW"

// === SERVICE SETUP ===
AT+QBTGATSS=0,1,6159,1                     // Primary service UUID 6159
AT+QBTGATSC=0,0,58,1,10777                 // Char 0: Read+Write+Notify
AT+QBTGATSCV=0,0,3,1,10777,244,"0000"      // Char 0 value, 244 bytes buffer
AT+QBTGATSCD=0,0,3,1,10498,2,"0300"        // Descriptor for notify

// === FINISH & ADVERTISE ===
AT+QBTGATSSC=1,1
AT+QBTADV=1

// === EXPECTED URCs AFTER PHONE CONNECTS ===
// +QBTGATSCON: 0,"<phone_mac>"
// +QBTLESTATE: 0,0,"<phone_mac>",1,18
// +QBTGATMTU: 0,247

// === WHEN PHONE WRITES CONFIG DATA ===
// +QBTLEVALDATA: 0,"<phone_mac>",<len>,"<hex_data>"

// === SEND RESPONSE / STATUS BACK TO PHONE ===
// AT+QBTGATSNOD=0,0,18,<len>,"<hex_data>"

// === STOP ADVERTISING AFTER CONNECTION ===
AT+QBTADV=0

// === ON DISCONNECT ===
// +QBTGATSDCON: 0,"<phone_mac>"
// Restart advertising if needed:
AT+QBTADV=1
```

---

## 12. Error Code Reference

| CME ERROR | Meaning | Fix |
|-----------|---------|-----|
| 4 | Operation not allowed (already in that state) | Normal if BLE already ON |
| 53 | Not found / context not ready | Power cycle: AT+QBTPWR=0 then AT+QBTPWR=1 |
| 58 | Not supported in this firmware build | Feature not compiled in firmware |

---

## 13. Notes for STM32 Firmware Implementation

- **UART parser:** STM32 must parse URCs asynchronously (interrupt-driven ring buffer)
- **URC vs response:** Normal AT responses come synchronously; URCs like `+QBTLEVALDATA` arrive any time
- **Hex decoding:** All data from BLE arrives as hex string — STM32 must convert hex string to binary
- **Config storage:** After receiving config via BLE, STM32 writes to NVM flash page
- **Advertising trigger:** Use GPIO button interrupt → send `AT+QBTADV=1` → start 60s timer → send `AT+QBTADV=0`
- **Shared UART:** BLE AT commands share USART2 with MQTT/GSM AT commands — use a mutex flag to avoid conflicts
- **Max characteristic value size:** 512 bytes per characteristic (set in `AT+QBTGATSCV`)
- **MTU negotiation:** Default MTU is 23 bytes. Exchange MTU to 220 for faster config transfers: `AT+QBTLEEXMTU=0,220`
