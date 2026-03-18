"""
MicroPython CC1101 433MHz Transceiver Driver for Raspberry Pi Pico (RP2040)

Wiring:
    CC1101 Pin  ->  Pico Pin
    VCC         ->  3.3V
    GND         ->  GND
    MOSI (SI)   ->  GP7 (SPI0 TX)
    MISO (SO)   ->  GP4 (SPI0 RX)
    SCK (SCLK)  ->  GP6 (SPI0 SCK)
    CSN (CS)    ->  GP5 (Chip Select)
    GDO0        ->  GP3 (Optional - for interrupts/status)
    GDO2        ->  GP2 (Optional - for status)
"""

from machine import Pin, SPI
import time
import rp2
from rp2 import PIO, StateMachine, asm_pio

# CC1101 Command Strobes
SRES    = 0x30  # Reset chip
SFSTXON = 0x31  # Enable/calibrate frequency synthesizer
SXOFF   = 0x32  # Turn off crystal oscillator
SCAL    = 0x33  # Calibrate frequency synthesizer
SRX     = 0x34  # Enable RX
STX     = 0x35  # Enable TX
SIDLE   = 0x36  # Exit RX/TX, turn off frequency synthesizer
SWOR    = 0x38  # Start automatic RX polling sequence
SPWD    = 0x39  # Enter power down mode
SFRX    = 0x3A  # Flush the RX FIFO buffer
SFTX    = 0x3B  # Flush the TX FIFO buffer
SWORRST = 0x3C  # Reset real time clock
SNOP    = 0x3D  # No operation

# CC1101 Configuration Registers
IOCFG2   = 0x00  # GDO2 output pin configuration
IOCFG1   = 0x01  # GDO1 output pin configuration
IOCFG0   = 0x02  # GDO0 output pin configuration
FIFOTHR  = 0x03  # RX FIFO and TX FIFO thresholds
SYNC1    = 0x04  # Sync word, high byte
SYNC0    = 0x05  # Sync word, low byte
PKTLEN   = 0x06  # Packet length
PKTCTRL1 = 0x07  # Packet automation control
PKTCTRL0 = 0x08  # Packet automation control
ADDR     = 0x09  # Device address
CHANNR   = 0x0A  # Channel number
FSCTRL1  = 0x0B  # Frequency synthesizer control
FSCTRL0  = 0x0C  # Frequency synthesizer control
FREQ2    = 0x0D  # Frequency control word, high byte
FREQ1    = 0x0E  # Frequency control word, middle byte
FREQ0    = 0x0F  # Frequency control word, low byte
MDMCFG4  = 0x10  # Modem configuration
MDMCFG3  = 0x11  # Modem configuration
MDMCFG2  = 0x12  # Modem configuration
MDMCFG1  = 0x13  # Modem configuration
MDMCFG0  = 0x14  # Modem configuration
DEVIATN  = 0x15  # Modem deviation setting
MCSM2    = 0x16  # Main Radio Control State Machine configuration
MCSM1    = 0x17  # Main Radio Control State Machine configuration
MCSM0    = 0x18  # Main Radio Control State Machine configuration
FOCCFG   = 0x19  # Frequency Offset Compensation configuration
BSCFG    = 0x1A  # Bit Synchronization configuration
AGCCTRL2 = 0x1B  # AGC control
AGCCTRL1 = 0x1C  # AGC control
AGCCTRL0 = 0x1D  # AGC control
WOREVT1  = 0x1E  # High byte Event 0 timeout
WOREVT0  = 0x1F  # Low byte Event 0 timeout
WORCTRL  = 0x20  # Wake On Radio control
FREND1   = 0x21  # Front end RX configuration
FREND0   = 0x22  # Front end TX configuration
FSCAL3   = 0x23  # Frequency synthesizer calibration
FSCAL2   = 0x24  # Frequency synthesizer calibration
FSCAL1   = 0x25  # Frequency synthesizer calibration
FSCAL0   = 0x26  # Frequency synthesizer calibration
RCCTRL1  = 0x27  # RC oscillator configuration
RCCTRL0  = 0x28  # RC oscillator configuration
FSTEST   = 0x29  # Frequency synthesizer calibration control
PTEST    = 0x2A  # Production test
AGCTEST  = 0x2B  # AGC test
TEST2    = 0x2C  # Various test settings
TEST1    = 0x2D  # Various test settings
TEST0    = 0x2E  # Various test settings

# Status Registers
PARTNUM  = 0x30  # Part number
VERSION  = 0x31  # Current version number
FREQEST  = 0x32  # Frequency offset estimate
LQI      = 0x33  # Demodulator estimate for Link Quality
RSSI     = 0x34  # Received signal strength indication
MARCSTATE= 0x35  # Control state machine state
WORTIME1 = 0x36  # High byte of WOR timer
WORTIME0 = 0x37  # Low byte of WOR timer
PKTSTATUS= 0x38  # Current GDOx status and packet status
VCO_VC_DAC=0x39  # Current setting from PLL calibration module
TXBYTES  = 0x3A  # Underflow and number of bytes in TX FIFO
RXBYTES  = 0x3B  # Overflow and number of bytes in RX FIFO

# FIFO Access
TXFIFO   = 0x3F  # TX FIFO (write)
RXFIFO   = 0x3F  # RX FIFO (read)
PATABLE  = 0x3E  # PA power table


class CC1101:
    """CC1101 Transceiver Driver for 433MHz operation"""
    
    # 433MHz configuration values (26MHz crystal)
    CONFIG_433MHZ = {
        IOCFG2:   0x29,  # GDO2 output pin config - chip ready
        IOCFG1:   0x2E,  # GDO1 output pin config - high impedance
        IOCFG0:   0x06,  # GDO0 output pin config - sync word
        FIFOTHR:  0x47,  # RX FIFO and TX FIFO thresholds
        SYNC1:    0xD3,  # Sync word, high byte
        SYNC0:    0x91,  # Sync word, low byte
        PKTLEN:   0xFF,  # Packet length (255 bytes max)
        PKTCTRL1: 0x04,  # Packet automation control
        PKTCTRL0: 0x05,  # Packet automation control - variable length
        ADDR:     0x00,  # Device address
        CHANNR:   0x00,  # Channel number
        FSCTRL1:  0x06,  # Frequency synthesizer control
        FSCTRL0:  0x00,  # Frequency synthesizer control
        FREQ2:    0x10,  # Frequency control word, high byte (433.92MHz)
        FREQ1:    0xB1,  # Frequency control word, middle byte
        FREQ0:    0x3B,  # Frequency control word, low byte
        MDMCFG4:  0xF8,  # Modem config - channel bandwidth
        MDMCFG3:  0x83,  # Modem config - data rate
        MDMCFG2:  0x33,  # Modem config - modulation format (ASK/OOK)
        MDMCFG1:  0x22,  # Modem config
        MDMCFG0:  0xF8,  # Modem config
        DEVIATN:  0x15,  # Modem deviation setting
        MCSM2:    0x07,  # Main Radio Control State Machine config
        MCSM1:    0x30,  # Main Radio Control State Machine config
        MCSM0:    0x18,  # Main Radio Control State Machine config
        FOCCFG:   0x16,  # Frequency Offset Compensation config
        BSCFG:    0x6C,  # Bit Synchronization config
        AGCCTRL2: 0x03,  # AGC control
        AGCCTRL1: 0x40,  # AGC control
        AGCCTRL0: 0x91,  # AGC control
        WOREVT1:  0x87,  # High byte Event 0 timeout
        WOREVT0:  0x6B,  # Low byte Event 0 timeout
        WORCTRL:  0xFB,  # Wake On Radio control
        FREND1:   0x56,  # Front end RX configuration
        FREND0:   0x11,  # Front end TX configuration (PA power index)
        FSCAL3:   0xE9,  # Frequency synthesizer calibration
        FSCAL2:   0x2A,  # Frequency synthesizer calibration
        FSCAL1:   0x00,  # Frequency synthesizer calibration
        FSCAL0:   0x1F,  # Frequency synthesizer calibration
        RCCTRL1:  0x41,  # RC oscillator configuration
        RCCTRL0:  0x00,  # RC oscillator configuration
        TEST2:    0x81,  # Various test settings
        TEST1:    0x35,  # Various test settings
        TEST0:    0x09,  # Various test settings
    }
    
    # PA Table for different power levels (433MHz)
    # Index 0 = lowest power, Index 7 = highest power (+10dBm)
    PA_TABLE = [0x00, 0x12, 0x0E, 0x34, 0x60, 0xC5, 0xC0, 0xC0]
    
    def __init__(self, spi_id=0, sck_pin=6, mosi_pin=7, miso_pin=4, cs_pin=5, gdo0_pin=3, gdo2_pin=2):
        """
        Initialize CC1101 transceiver
        
        Args:
            spi_id: SPI bus ID (0 or 1)
            sck_pin: SPI clock pin
            mosi_pin: SPI MOSI pin
            miso_pin: SPI MISO pin
            cs_pin: Chip select pin
            gdo0_pin: GDO0 pin (optional)
            gdo2_pin: GDO2 pin (optional)
        """
        # Initialize pins
        self.cs = Pin(cs_pin, Pin.OUT)
        self.cs.value(1)  # Deselect
        
        self.gdo0 = Pin(gdo0_pin, Pin.IN) if gdo0_pin else None
        self.gdo2 = Pin(gdo2_pin, Pin.IN) if gdo2_pin else None
        
        # Store MISO pin for ready check
        self.miso_pin = Pin(miso_pin, Pin.IN)
        
        # Initialize SPI - slower speed for reliability (500kHz)
        # CC1101 supports up to 6.5MHz but lower is more reliable
        self.spi = SPI(spi_id, baudrate=500000, polarity=0, phase=0,
                       sck=Pin(sck_pin), mosi=Pin(mosi_pin), miso=Pin(miso_pin))
        
        self._initialized = False
    
    def test_spi(self):
        """Test SPI communication by writing and reading back a register"""
        print("Testing SPI communication...")
        
        # Write a test value to SYNC1 register
        test_val = 0xAA
        self.write_register(SYNC1, test_val)
        time.sleep_ms(1)
        read_val = self.read_register(SYNC1)
        
        print(f"  Wrote: 0x{test_val:02X}, Read: 0x{read_val:02X}")
        
        if read_val == test_val:
            print("  SPI communication OK!")
            return True
        else:
            print("  SPI communication FAILED!")
            # Try reading multiple times
            for i in range(3):
                read_val = self.read_register(SYNC1)
                print(f"  Retry {i+1}: 0x{read_val:02X}")
            return False
    
    def _spi_transfer(self, data):
        """Transfer data over SPI and return received data"""
        result = bytearray(len(data))
        self.spi.write_readinto(bytes(data), result)
        return result
    
    def _select(self):
        """Select the CC1101 (CS low) and wait for MISO ready"""
        self.cs.value(0)
        # Wait for MISO to go low (chip ready) - up to 100ms
        timeout = 10000
        while self.miso_pin.value() and timeout > 0:
            time.sleep_us(10)
            timeout -= 1
        if timeout <= 0:
            print("Warning: MISO never went low (chip not ready)")
        time.sleep_us(10)
    
    def _deselect(self):
        """Deselect the CC1101 (CS high)"""
        time.sleep_us(10)
        self.cs.value(1)
        time.sleep_us(50)  # Longer delay between transactions
    
    def strobe(self, cmd):
        """Send a command strobe and return status byte"""
        self._select()
        result = self._spi_transfer([cmd])
        self._deselect()
        return result[0]
    
    def read_register(self, addr):
        """Read a single configuration register"""
        self._select()
        result = self._spi_transfer([addr | 0x80, 0x00])
        self._deselect()
        return result[1]
    
    def read_status_register(self, addr):
        """Read a status register"""
        self._select()
        result = self._spi_transfer([addr | 0xC0, 0x00])
        self._deselect()
        return result[1]
    
    def write_register(self, addr, value):
        """Write to a single configuration register"""
        self._select()
        self._spi_transfer([addr, value])
        self._deselect()
    
    def write_burst(self, addr, data):
        """Write multiple bytes in burst mode"""
        self._select()
        self._spi_transfer([addr | 0x40] + list(data))
        self._deselect()
    
    def read_burst(self, addr, length):
        """Read multiple bytes in burst mode"""
        self._select()
        result = self._spi_transfer([addr | 0xC0] + [0x00] * length)
        self._deselect()
        return result[1:]
    
    def reset(self):
        """Reset the CC1101"""
        # Manual reset sequence per datasheet
        self.cs.value(1)
        time.sleep_us(50)
        self.cs.value(0)
        time.sleep_us(50)
        self.cs.value(1)
        time.sleep_us(50)
        
        # Send reset strobe
        self._select()
        self._spi_transfer([SRES])
        self._deselect()
        
        time.sleep_ms(100)  # Wait for reset to complete
        
        # Wait for chip ready by sending SNOP and checking response
        ready = False
        for i in range(100):
            self._select()
            result = self._spi_transfer([SNOP])
            self._deselect()
            status = result[0]
            if (status & 0x80) == 0:  # Chip ready when bit 7 is 0
                ready = True
                print(f"Chip ready after {i+1} attempts, status: 0x{status:02X}")
                break
            time.sleep_ms(10)
        
        if not ready:
            print("Warning: Chip did not become ready after reset")
    
    def init(self, frequency_mhz=433.92, power_level=7):
        """
        Initialize CC1101 for 433MHz operation
        
        Args:
            frequency_mhz: Frequency in MHz (default 433.92)
            power_level: TX power level 0-7 (default 7 = max)
        """
        self.reset()
        
        # Test SPI communication first
        if not self.test_spi():
            print("SPI test failed - check wiring!")
            return False
        
        # Write configuration registers
        for reg, value in self.CONFIG_433MHZ.items():
            self.write_register(reg, value)
        
        # Set frequency
        self.set_frequency(frequency_mhz)
        
        # Set PA table for power level
        self.set_tx_power(power_level)
        
        # Calibrate
        self.strobe(SCAL)
        time.sleep_ms(1)
        
        # Enter idle mode
        self.strobe(SIDLE)
        
        self._initialized = True
        
        # Verify initialization
        part_num = self.read_status_register(PARTNUM)
        version = self.read_status_register(VERSION)
        print(f"CC1101 initialized - Part: 0x{part_num:02X}, Version: 0x{version:02X}")
        
        # Debug: Check frequency registers were written correctly
        freq2 = self.read_register(FREQ2)
        freq1 = self.read_register(FREQ1)
        freq0 = self.read_register(FREQ0)
        print(f"Frequency registers: FREQ2=0x{freq2:02X}, FREQ1=0x{freq1:02X}, FREQ0=0x{freq0:02X}")
        print(f"State after init: {self.get_state()}")
        
        # Version 0x00 might indicate SPI issues, but some clones report 0x00
        if version == 0x00:
            print("Warning: Version 0x00 detected - check SPI wiring or possible CC1101 clone")
        
        return True  # Continue even if version check fails (some clones report 0x00)
    
    def set_frequency(self, freq_mhz):
        """
        Set the operating frequency
        
        Args:
            freq_mhz: Frequency in MHz (e.g., 433.92)
        """
        # Calculate frequency register values
        # FREQ = (freq_mhz * 2^16) / 26 MHz (crystal frequency)
        freq_val = int((freq_mhz * 65536) / 26.0)
        
        self.write_register(FREQ2, (freq_val >> 16) & 0xFF)
        self.write_register(FREQ1, (freq_val >> 8) & 0xFF)
        self.write_register(FREQ0, freq_val & 0xFF)
    
    def set_tx_power(self, level):
        """
        Set TX power level
        
        Args:
            level: Power level 0-7 (7 = max ~+10dBm)
        """
        if level < 0 or level > 7:
            level = 7
        
        # Write PA table with selected power level
        pa_value = self.PA_TABLE[level]
        self.write_register(PATABLE, pa_value)
    
    def set_channel(self, channel):
        """
        Set the channel number
        
        Args:
            channel: Channel number 0-255
        """
        self.write_register(CHANNR, channel & 0xFF)
    
    def set_sync_word(self, sync_word):
        """
        Set sync word (16-bit)
        
        Args:
            sync_word: 16-bit sync word
        """
        self.write_register(SYNC1, (sync_word >> 8) & 0xFF)
        self.write_register(SYNC0, sync_word & 0xFF)
    
    def get_state(self):
        """Get the current state machine state"""
        state = self.read_status_register(MARCSTATE) & 0x1F
        states = {
            0x00: "SLEEP",
            0x01: "IDLE",
            0x02: "XOFF",
            0x03: "VCOON_MC",
            0x04: "REGON_MC",
            0x05: "MANCAL",
            0x06: "VCOON",
            0x07: "REGON",
            0x08: "STARTCAL",
            0x09: "BWBOOST",
            0x0A: "FS_LOCK",
            0x0B: "IFADCON",
            0x0C: "ENDCAL",
            0x0D: "RX",
            0x0E: "RX_END",
            0x0F: "RX_RST",
            0x10: "TXRX_SWITCH",
            0x11: "RXFIFO_OVERFLOW",
            0x12: "FSTXON",
            0x13: "TX",
            0x14: "TX_END",
            0x15: "RXTX_SWITCH",
            0x16: "TXFIFO_UNDERFLOW",
        }
        return states.get(state, f"UNKNOWN (0x{state:02X})")
    
    def get_rssi(self):
        """Get current RSSI value in dBm"""
        rssi_raw = self.read_status_register(RSSI)
        if rssi_raw >= 128:
            rssi_dbm = (rssi_raw - 256) / 2 - 74
        else:
            rssi_dbm = rssi_raw / 2 - 74
        return rssi_dbm
    
    def transmit(self, data):
        """
        Transmit data packet
        
        Args:
            data: Bytes or bytearray to transmit (max 61 bytes for variable length)
        
        Returns:
            True if transmission successful, False otherwise
        """
        if not self._initialized:
            print("Error: CC1101 not initialized")
            return False
        
        if len(data) > 61:
            print("Error: Data too long (max 61 bytes)")
            return False
        
        # Ensure we're in IDLE state
        self.strobe(SIDLE)
        time.sleep_us(100)
        
        # Flush TX FIFO
        self.strobe(SFTX)
        time.sleep_us(100)
        
        # Write length byte and data to TX FIFO
        tx_data = bytes([len(data)]) + bytes(data)
        self.write_burst(TXFIFO, tx_data)
        
        # Verify data was written
        tx_bytes = self.read_status_register(TXBYTES) & 0x7F
        print(f"TX FIFO bytes: {tx_bytes}")
        
        # Start transmission
        self.strobe(STX)
        time.sleep_ms(1)
        
        # Wait for transmission to complete
        timeout = 100  # 100ms timeout (shorter for debugging)
        while timeout > 0:
            state = self.get_state()
            tx_bytes = self.read_status_register(TXBYTES) & 0x7F
            print(f"State: {state}, TX bytes remaining: {tx_bytes}")
            
            if state == "IDLE":
                if tx_bytes == 0:
                    return True  # Success - all bytes transmitted
                break
            if state == "TX_END":
                time.sleep_ms(5)
                self.strobe(SIDLE)
                return True
            if state == "TXFIFO_UNDERFLOW":
                print("TX FIFO underflow!")
                self.strobe(SFTX)
                self.strobe(SIDLE)
                return False
            time.sleep_ms(10)
            timeout -= 10
        
        if timeout <= 0:
            print("Transmission timeout")
            return False
        
        return True
    
    def transmit_raw(self, data):
        """
        Transmit raw data without length byte (for fixed length mode)
        
        Args:
            data: Bytes or bytearray to transmit
        
        Returns:
            True if transmission successful, False otherwise
        """
        if not self._initialized:
            print("Error: CC1101 not initialized")
            return False
        
        # Ensure we're in IDLE state
        self.strobe(SIDLE)
        time.sleep_us(100)
        
        # Flush TX FIFO
        self.strobe(SFTX)
        time.sleep_us(100)
        
        # Write data to TX FIFO
        self.write_burst(TXFIFO, data)
        
        # Start transmission
        self.strobe(STX)
        
        # Wait for GDO0 to go high (sync word transmitted) then low (packet sent)
        timeout = 1000
        while timeout > 0:
            state = self.get_state()
            if state == "IDLE":
                break
            time.sleep_ms(1)
            timeout -= 1
        
        return timeout > 0
    
    def receive(self, timeout_ms=1000):
        """
        Receive a data packet
        
        Args:
            timeout_ms: Receive timeout in milliseconds
        
        Returns:
            Received data as bytes, or None if no data received
        """
        if not self._initialized:
            print("Error: CC1101 not initialized")
            return None
        
        # Ensure we're in IDLE state
        self.strobe(SIDLE)
        time.sleep_us(100)
        
        # Flush RX FIFO
        self.strobe(SFRX)
        time.sleep_us(100)
        
        # Enter RX mode
        self.strobe(SRX)
        
        # Wait for packet
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
            rx_bytes = self.read_status_register(RXBYTES) & 0x7F
            if rx_bytes > 0:
                # Read length byte
                length = self.read_register(RXFIFO)
                if length > 0 and length <= 61:
                    # Read data
                    data = self.read_burst(RXFIFO, length)
                    # Read status bytes (RSSI and LQI)
                    status = self.read_burst(RXFIFO, 2)
                    
                    # Return to IDLE
                    self.strobe(SIDLE)
                    return bytes(data)
            
            time.sleep_ms(1)
        
        # Timeout - return to IDLE
        self.strobe(SIDLE)
        return None
    
    def enter_sleep(self):
        """Enter low power sleep mode"""
        self.strobe(SIDLE)
        self.strobe(SPWD)
    
    def wake_up(self):
        """Wake up from sleep mode"""
        self._select()
        time.sleep_us(10)
        self._deselect()
        time.sleep_ms(1)
        self.strobe(SIDLE)
    
    def set_ask_ook_mode(self):
        """
        Configure CC1101 for raw ASK/OOK transmission with NRZ encoding.
        Uses asynchronous serial mode - GDO0 becomes the TX data input.
        """
        self.strobe(SIDLE)
        time.sleep_ms(1)
        
        # PKTCTRL0: Asynchronous serial mode (no packet handling)
        # Bits [5:4] = 11 = Asynchronous serial mode
        self.write_register(PKTCTRL0, 0x30)
        
        # MDMCFG2: ASK/OOK modulation, no sync word
        # Bits [6:4] = 011 = ASK/OOK
        # Bits [2:0] = 000 = No preamble/sync
        self.write_register(MDMCFG2, 0x30)
        
        # FREND0: Set PA ramping for ASK (index 0 = off, index 1 = on)
        self.write_register(FREND0, 0x11)
        
        # Set PA table for ASK: index 0 = 0 (off), index 1 = max power (on)
        self._select()
        self._spi_transfer([PATABLE | 0x40, 0x00, 0xC0])  # Burst write 2 bytes
        self._deselect()
        
        # IOCFG0: GDO0 as serial TX data input (0x2D = serial data output, inverted for TX input)
        self.write_register(IOCFG0, 0x2D)
        
        print("ASK/OOK mode configured - use GDO0 (GP3) for TX data")
    
    def transmit_bits_ask(self, bits, bit_duration_us=350):
        """
        Transmit raw bits using ASK/OOK modulation (NRZ encoding).
        
        Args:
            bits: String of '1' and '0' or list of integers (1/0)
            bit_duration_us: Duration of each bit in microseconds (default 350us)
        
        Example:
            cc1101.transmit_bits_ask("101010101010", bit_duration_us=350)
        """
        if not self._initialized:
            print("Error: CC1101 not initialized")
            return False
        
        # Convert to list if string
        if isinstance(bits, str):
            bits = [int(b) for b in bits if b in '01']
        
        # Ensure GDO0 is configured as output for TX data
        gdo0_out = Pin(3, Pin.OUT)
        gdo0_out.value(0)
        
        # Enter TX mode
        self.strobe(SIDLE)
        time.sleep_us(100)
        self.strobe(STX)
        time.sleep_us(500)  # Wait for TX to stabilize
        
        # Transmit each bit (NRZ: 1=carrier on, 0=carrier off)
        for bit in bits:
            gdo0_out.value(bit)
            time.sleep_us(bit_duration_us)
        
        # End with carrier off
        gdo0_out.value(0)
        time.sleep_us(100)
        
        # Return to idle
        self.strobe(SIDLE)
        
        # Restore GDO0 as input
        self.gdo0 = Pin(3, Pin.IN)
        
        return True
    
    def transmit_pattern_ask(self, pattern, repeats=5, gap_ms=10, bit_duration_us=350):
        """
        Transmit a bit pattern multiple times (common for remotes).
        
        Args:
            pattern: String of '1' and '0' representing the code
            repeats: Number of times to repeat the pattern
            gap_ms: Gap between repetitions in milliseconds
            bit_duration_us: Duration of each bit in microseconds
        
        Example:
            # Typical garage door pattern
            cc1101.transmit_pattern_ask("101011001010110010101100", repeats=10)
        """
        print(f"Transmitting pattern {repeats} times...")
        
        for i in range(repeats):
            self.transmit_bits_ask(pattern, bit_duration_us)
            if i < repeats - 1:
                time.sleep_ms(gap_ms)
        
        print("Transmission complete")
        return True
    
    def transmit_pwm_ask(self, bits, short_us=350, long_us=1050, bit_duration_us=1400):
        """
        Transmit using PWM encoding (common for PT2262, EV1527 remotes).
        
        In PWM: 
        - '0' = short high + long low
        - '1' = long high + short low
        
        Args:
            bits: String of '1' and '0'
            short_us: Short pulse duration (microseconds)
            long_us: Long pulse duration (microseconds)  
            bit_duration_us: Total bit period (short + long)
        """
        if isinstance(bits, str):
            bits = [int(b) for b in bits if b in '01']
        
        gdo0_out = Pin(3, Pin.OUT)
        gdo0_out.value(0)
        
        self.strobe(SIDLE)
        time.sleep_us(100)
        self.strobe(STX)
        time.sleep_us(500)
        
        for bit in bits:
            if bit == 1:
                # Long high, short low
                gdo0_out.value(1)
                time.sleep_us(long_us)
                gdo0_out.value(0)
                time.sleep_us(short_us)
            else:
                # Short high, long low
                gdo0_out.value(1)
                time.sleep_us(short_us)
                gdo0_out.value(0)
                time.sleep_us(long_us)
        
        gdo0_out.value(0)
        self.strobe(SIDLE)
        self.gdo0 = Pin(3, Pin.IN)
        
        return True


# PIO program for precise bit timing - simpler version
@asm_pio(out_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT)
def ask_tx_pio():
    """PIO program to output bits with precise timing"""
    pull(block)            # Wait for data
    label("bitloop")
    out(pins, 1)           # Output 1 bit to pin
    jmp(not_osre, "bitloop")  # Loop until shift register empty
    

class ASKTransmitter:
    """
    Precise ASK transmitter using RP2040 PIO for accurate timing.
    """
    
    def __init__(self, cc1101, pin=3, bit_duration_us=350):
        """
        Initialize ASK transmitter with PIO.
        
        Args:
            cc1101: CC1101 instance (already initialized)
            pin: GPIO pin for TX data (GDO0)
            bit_duration_us: Duration of each bit in microseconds
        """
        self.cc1101 = cc1101
        self.pin_num = pin
        self.bit_duration_us = bit_duration_us
        self.pin = Pin(pin, Pin.OUT)
        self.pin.value(0)
        self.sm = None
    
    def _get_freq(self, bit_duration_us):
        """Calculate PIO frequency for desired bit duration"""
        # PIO program is 2 instructions per bit (out + jmp)
        # freq = instructions_per_second = 2 / bit_duration
        return int(2_000_000 / bit_duration_us)
    
    def transmit(self, bits, repeats=1, gap_ms=10):
        """
        Transmit bits with precise timing using PIO.
        
        Args:
            bits: String of '1' and '0'
            repeats: Number of times to repeat
            gap_ms: Gap between repeats in ms
        """
        # Convert bit string to list
        if isinstance(bits, str):
            bit_list = [int(b) for b in bits if b in '01']
        else:
            bit_list = list(bits)
        
        # Calculate frequency
        freq = self._get_freq(self.bit_duration_us)
        print(f"PIO freq: {freq} Hz for {self.bit_duration_us}us bits")
        
        # Create state machine
        sm = StateMachine(0, ask_tx_pio, freq=freq, out_base=self.pin)
        
        # Put CC1101 in TX mode
        self.cc1101.strobe(SIDLE)
        time.sleep_us(100)
        self.cc1101.strobe(STX)
        time.sleep_ms(1)
        
        for rep in range(repeats):
            # Pack bits into 32-bit words (LSB first)
            # Pad with zeros to ensure clean ending
            padded_bits = bit_list + [0] * 8
            
            sm.active(1)
            
            # Send 32 bits at a time
            i = 0
            while i < len(padded_bits):
                word = 0
                for j in range(32):
                    if i + j < len(padded_bits):
                        word |= (padded_bits[i + j] << j)
                sm.put(word)
                i += 32
            
            # Wait for all bits to be transmitted
            time.sleep_us(len(padded_bits) * self.bit_duration_us + 500)
            
            sm.active(0)
            
            # Ensure pin low between repeats
            self.pin.value(0)
            
            if rep < repeats - 1:
                time.sleep_ms(gap_ms)
        
        # Return to idle
        self.cc1101.strobe(SIDLE)
        self.pin.value(0)
        print(f"Transmitted {len(bit_list)} bits x {repeats} times")


def transmit_ask_precise(cc1101, bits, bit_duration_us=350, repeats=10, gap_ms=10):
    """
    Simple precise transmission using tight loop with timing compensation.
    Calibrated for RP2040 at 125MHz.
    """
    if isinstance(bits, str):
        bit_list = [int(b) for b in bits if b in '01']
    else:
        bit_list = list(bits)
    
    gdo0 = Pin(3, Pin.OUT)
    gdo0.value(0)
    
    # Enter TX mode
    cc1101.strobe(SIDLE)
    time.sleep_us(100)
    cc1101.strobe(STX)
    time.sleep_ms(1)
    
    # Compensate for loop overhead (~100us based on your capture showing 2x duration)
    # Actual sleep should be about half to account for Python overhead
    adjusted_duration = max(50, bit_duration_us - 180)
    
    print(f"Transmitting {len(bit_list)} bits, adjusted duration: {adjusted_duration}us")
    
    for rep in range(repeats):
        for bit in bit_list:
            gdo0.value(bit)
            time.sleep_us(adjusted_duration)
        
        gdo0.value(0)
        
        if rep < repeats - 1:
            time.sleep_ms(gap_ms)
    
    cc1101.strobe(SIDLE)
    gdo0.value(0)
    print("Done")


# Example usage
def main():
    print("CC1101 433MHz Transmitter Example")
    print("-" * 40)
    
    # Initialize CC1101
    cc1101 = CC1101(
        spi_id=0,
        sck_pin=6,
        mosi_pin=7,
        miso_pin=4,
        cs_pin=5,
        gdo0_pin=3,
        gdo2_pin=2
    )
    
    # Initialize for 433.92 MHz with max power
    if not cc1101.init(frequency_mhz=433.92, power_level=7):
        print("Failed to initialize CC1101!")
        return
    
    print(f"Current state: {cc1101.get_state()}")
    print(f"RSSI: {cc1101.get_rssi():.1f} dBm")
    print()
    
    # Transmit some test data
    test_messages = [
        b"Hello 433MHz!",
        b"Test message 1",
        b"Test message 2",
        bytes([0x01, 0x02, 0x03, 0x04, 0x05]),  # Binary data
    ]
    
    for i, msg in enumerate(test_messages):
        print(f"Transmitting message {i+1}: {msg}")
        if cc1101.transmit(msg):
            print("  -> Transmission successful!")
        else:
            print("  -> Transmission failed!")
        time.sleep_ms(500)
    
    print()
    print("Transmission complete!")
    
    # Optional: Enter receive mode and wait for responses
    print("\nEntering receive mode for 5 seconds...")
    received = cc1101.receive(timeout_ms=5000)
    if received:
        print(f"Received: {received}")
    else:
        print("No data received")
    
    # Enter sleep mode when done
    cc1101.enter_sleep()
    print("CC1101 entered sleep mode")


if __name__ == "__main__":
    main()
