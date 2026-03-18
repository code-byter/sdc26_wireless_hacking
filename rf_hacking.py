from machine import Pin
import time
from cc1101_433mhz import CC1101, transmit_ask_precise, SIDLE, SRX, IOCFG0, PKTCTRL0, MDMCFG4, MDMCFG3, MDMCFG2, AGCCTRL2, AGCCTRL1, AGCCTRL0

# ============================================================
# Configuration
# ============================================================
# Timing for PT2262
BIT_DURATION_US = 350     # Base bit duration
EXPECTED_SHORT_US = 350   # Short pulse ~350us
EXPECTED_LONG_US = 1050   # Long pulse ~1050us (3x short)

# Sniffer config
RX_PIN_NUM = 3            # GDO0 is connected to GP3
MAX_SAMPLES = 4000
MIN_PULSE_US = 200
MAX_PULSE_US = 15000
RSSI_THRESHOLD = -75      # dBm

# Global sniffer state
timestamps = [0] * MAX_SAMPLES
levels = [0] * MAX_SAMPLES
edge_count = 0
collecting = False
rx_pin = None

# ============================================================
# PT2262 Encoding/Decoding
# ============================================================

def encode_pt2262(address, button, bits=24):
    """
    Encode a PT2262-style remote signal.
    
    Args:
        address: Device address/ID (20-bit)
        button: Button code (4-bit)
        bits: Total data bits (default 24)
    
    Returns:
        Raw bit pattern string for transmission
    """
    # Combine address and button into full code
    code = ((address & 0xFFFFF) << 4) | (button & 0xF)
    
    # Convert each bit to tri-state PWM encoding
    # 1 = 1110 (long high, short low)
    # 0 = 1000 (short high, long low)
    pattern = ""
    for i in range(bits - 1, -1, -1):
        bit = (code >> i) & 1
        if bit:
            pattern += "1110"
        else:
            pattern += "1000"
    
    # Add stop bit (sync pulse)
    pattern += "1"
    
    return pattern


def decode_pt2262(bits_str):
    """
    Decode a PT2262 bit string to address and button.
    
    Args:
        bits_str: String of '0', '1', 'F', '?' characters (24 bits)
    
    Returns:
        Tuple of (address, button, hex_code) or None if invalid
    """
    # Clean up and validate
    clean_bits = bits_str.replace('F', '0').replace('?', '0')
    
    if len(clean_bits) < 24:
        return None
    
    # Take first 24 bits
    code_24 = clean_bits[:24]
    
    try:
        full_code = int(code_24, 2)
        address = int(code_24[:20], 2)
        button = int(code_24[20:24], 2)
        return (address, button, full_code)
    except:
        return None


# ============================================================
# Sniffer Functions
# ============================================================

def rx_callback(pin):
    global edge_count, collecting
    if collecting and edge_count < MAX_SAMPLES:
        timestamps[edge_count] = time.ticks_us()
        levels[edge_count] = pin.value()
        edge_count += 1


def decode_segment(segment, threshold):
    """
    Decode a single segment of HIGH/LOW pulse pairs into PT2262 bits.
    Tries both aligned and offset-by-1 starting positions.
    
    Returns:
        Best decoded bit string, or "" if nothing decoded.
    """
    candidates = []
    
    for offset in range(2):  # Try starting at index 0 and 1
        bits = ""
        i = offset
        while i < len(segment) - 1:
            dur1, lvl1 = segment[i]
            dur2, lvl2 = segment[i + 1]
            
            if lvl1 == 1 and lvl2 == 0:
                high_dur = dur1
                low_dur = dur2
                
                if high_dur < threshold and low_dur > threshold:
                    bits += "0"
                elif high_dur > threshold and low_dur < threshold:
                    bits += "1"
                elif high_dur < threshold and low_dur < threshold:
                    bits += "F"
                else:
                    bits += "?"
                i += 2
            else:
                i += 1
        
        if bits:
            candidates.append(bits)
    
    if not candidates:
        return ""
    
    # Prefer the candidate closest to 24 bits
    candidates.sort(key=lambda b: abs(len(b) - 24))
    return candidates[0]


def compute_adaptive_threshold(valid_pulses):
    """
    Compute adaptive short/long threshold from actual pulse durations.
    Uses iterative k-means clustering on non-sync pulses.
    """
    data_durations = [d for d, l in valid_pulses if d < 3000]
    
    if len(data_durations) < 10:
        return (EXPECTED_SHORT_US + EXPECTED_LONG_US) // 2
    
    # Seed with expected values
    short_center = EXPECTED_SHORT_US
    long_center = EXPECTED_LONG_US
    
    # 3 iterations of k-means
    for _ in range(3):
        short_sum, short_n = 0, 0
        long_sum, long_n = 0, 0
        mid = (short_center + long_center) // 2
        
        for d in data_durations:
            if d < mid:
                short_sum += d
                short_n += 1
            else:
                long_sum += d
                long_n += 1
        
        if short_n > 0:
            short_center = short_sum // short_n
        if long_n > 0:
            long_center = long_sum // long_n
    
    threshold = (short_center + long_center) // 2
    return threshold


def analyze_signal(edges, lvls, count):
    """
    Analyze captured signal and decode PT2262 frame.
    
    Decodes ALL segments between sync gaps and uses majority voting
    to pick the most reliable 24-bit frame.
    """
    if count < 10:
        print(f"Too few edges ({count})")
        return None
        
    print(f"\n{'='*50}")
    print(f"Captured {count} transitions")
    
    # Calculate pulse durations with level info
    pulse_info = []
    for i in range(count - 1):
        dt = time.ticks_diff(edges[i+1], edges[i])
        pulse_info.append((dt, lvls[i]))
    
    # Filter noise
    valid_pulses = [(d, l) for d, l in pulse_info if MIN_PULSE_US <= d <= MAX_PULSE_US]
    
    if len(valid_pulses) < 10:
        print(f"Too few valid pulses ({len(valid_pulses)})")
        return None
    
    valid_durations = [d for d, l in valid_pulses]
    
    print(f"Valid pulses: {len(valid_durations)}")
    print(f"Pulse range: {min(valid_durations)} - {max(valid_durations)} us")
    
    # Adaptive threshold from actual pulse clustering
    threshold = compute_adaptive_threshold(valid_pulses)
    print(f"Adaptive threshold: {threshold} us")
    
    # Find sync gaps to segment transmissions
    sync_indices = [i for i, (d, l) in enumerate(valid_pulses) if d > 5000]
    
    if sync_indices:
        print(f"Found {len(sync_indices)} sync gaps")
    
    # Build segments between sync gaps
    boundaries = [-1] + sync_indices + [len(valid_pulses)]
    segments = []
    for j in range(len(boundaries) - 1):
        start = boundaries[j] + 1
        end = boundaries[j + 1]
        if end - start >= 20:  # Need enough pulses for a frame
            segments.append(valid_pulses[start:end])
    
    # Decode every segment
    all_decoded = []
    valid_24 = []
    
    for seg_idx, segment in enumerate(segments):
        bits = decode_segment(segment, threshold)
        if bits:
            tag = ""
            if len(bits) == 24 and '?' not in bits:
                valid_24.append(bits)
                tag = " [OK]"
            elif len(bits) == 24:
                valid_24.append(bits)
                tag = " [OK?]"
            all_decoded.append(bits)
            print(f"  Seg {seg_idx}: ({len(bits):2d}b) {bits}{tag}")
    
    # Majority vote on valid 24-bit frames
    if valid_24:
        vote_counts = {}
        for frame in valid_24:
            clean = frame.replace('?', '0').replace('F', '0')
            vote_counts[clean] = vote_counts.get(clean, 0) + 1
        
        best_frame = max(vote_counts, key=vote_counts.get)
        confidence = vote_counts[best_frame]
        print(f"\nMajority vote: {best_frame} ({confidence}/{len(valid_24)} frames agree)")
        
        result = decode_pt2262(best_frame)
        if result:
            address, button, full_code = result
            print(f"\n>>> PT2262 Frame Decoded <<<")
            print(f"    Full Code: 0x{full_code:06X}")
            print(f"    Address:   0x{address:05X}")
            print(f"    Button:    0x{button:X}")
            print(f"    Confidence: {confidence}/{len(valid_24)}")
            return result
    
    # Fallback: try bit-length-tolerant decode (23 or 25 bits)
    print("\nNo exact 24-bit frames; trying close matches...")
    close_matches = [b for b in all_decoded if 23 <= len(b) <= 25 and b.count('?') < 3]
    
    if close_matches:
        # Normalize to 24 bits
        normalized = {}
        for bits in close_matches:
            clean = bits.replace('?', '0').replace('F', '0')
            if len(clean) == 23:
                clean = '0' + clean  # Likely missed first bit
            elif len(clean) == 25:
                clean = clean[1:]   # Likely extra leading bit
            
            if len(clean) == 24:
                normalized[clean] = normalized.get(clean, 0) + 1
        
        if normalized:
            best = max(normalized, key=normalized.get)
            conf = normalized[best]
            print(f"Best close match: {best} ({conf} frames)")
            
            result = decode_pt2262(best)
            if result:
                address, button, full_code = result
                print(f"\n>>> PT2262 Frame Decoded (approx) <<<")
                print(f"    Full Code: 0x{full_code:06X}")
                print(f"    Address:   0x{address:05X}")
                print(f"    Button:    0x{button:X}")
                print(f"    Confidence: ~{conf}/{len(close_matches)}")
                return result
    
    print("Could not decode signal")
    return None


def setup_rx_mode(cc1101):
    """Configure CC1101 for ASK/OOK receive mode."""
    cc1101.strobe(SIDLE)
    time.sleep_ms(10)
    
    cc1101.write_register(IOCFG0, 0x0D)
    cc1101.write_register(PKTCTRL0, 0x32)
    cc1101.write_register(MDMCFG4, 0x86)
    cc1101.write_register(MDMCFG3, 0x83)
    cc1101.write_register(MDMCFG2, 0x30)
    cc1101.write_register(AGCCTRL2, 0x03)
    cc1101.write_register(AGCCTRL1, 0x00)
    cc1101.write_register(AGCCTRL0, 0x91)
    
    cc1101.strobe(SRX)
    time.sleep_ms(10)


def sniff(cc1101, timeout_ms=None):
    """
    Sniff for PT2262 signals.
    
    Args:
        cc1101: Initialized CC1101 instance
        timeout_ms: Optional timeout in ms (None = infinite)
    
    Returns:
        Decoded (address, button, code) tuple or None
    """
    global collecting, edge_count, rx_pin
    
    print("\n[SNIFFER] Configuring for ASK/OOK receive...")
    setup_rx_mode(cc1101)
    
    rx_pin = Pin(RX_PIN_NUM, Pin.IN)
    
    print(f"[SNIFFER] RSSI Threshold: {RSSI_THRESHOLD} dBm")
    print("[SNIFFER] Waiting for signal... (Ctrl+C to stop)")
    
    start_time = time.ticks_ms()
    result = None
    
    try:
        while True:
            # Check timeout
            if timeout_ms and time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                print("[SNIFFER] Timeout")
                break
            
            rssi = cc1101.get_rssi()
            
            if rssi > RSSI_THRESHOLD:
                print(f"\n[SNIFFER] Signal detected! RSSI: {rssi:.1f} dBm")
                
                edge_count = 0
                collecting = True
                rx_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=rx_callback)
                
                signal_end_time = None
                while True:
                    rssi = cc1101.get_rssi()
                    
                    if rssi < RSSI_THRESHOLD - 5:
                        if signal_end_time is None:
                            signal_end_time = time.ticks_ms()
                        elif time.ticks_diff(time.ticks_ms(), signal_end_time) > 200:
                            break
                    else:
                        signal_end_time = None
                    
                    if edge_count >= MAX_SAMPLES:
                        print("[SNIFFER] Buffer full!")
                        break
                    
                    time.sleep_ms(5)
                
                collecting = False
                rx_pin.irq(handler=None)
                
                if edge_count > 0:
                    data = list(timestamps[:edge_count])
                    lvl_data = list(levels[:edge_count])
                    result = analyze_signal(data, lvl_data, edge_count)
                    if result:
                        break  # Got valid decode, exit
                
                print("\n[SNIFFER] Waiting for signal...")
            
            time.sleep_ms(50)
            
    except KeyboardInterrupt:
        print("\n[SNIFFER] Stopped by user")
    finally:
        if rx_pin:
            rx_pin.irq(handler=None)
        cc1101.strobe(SIDLE)
    
    return result


# ============================================================
# Transmit Functions
# ============================================================

def transmit(cc1101, address, button, repeats=10):
    """
    Transmit a PT2262 code.
    
    Args:
        cc1101: CC1101 instance
        address: 20-bit address
        button: 4-bit button code
        repeats: Number of repetitions
    """
    cc1101.set_ask_ook_mode()
    
    pattern = encode_pt2262(address, button)
    full_code = (address << 4) | button
    
    print(f"\n[TX] Transmitting PT2262 code")
    print(f"     Address: 0x{address:05X}")
    print(f"     Button:  0x{button:X}")
    print(f"     Code:    0x{full_code:06X}")
    
    transmit_ask_precise(cc1101, pattern, bit_duration_us=BIT_DURATION_US, repeats=repeats, gap_ms=10)
    print("[TX] Done")


# ============================================================
# Main / Interactive Menu
# ============================================================

def main():
    cc1101 = CC1101()
    cc1101.init()

    # ToDo: Add code for sniffing and replaying

if __name__ == "__main__":
    main()
