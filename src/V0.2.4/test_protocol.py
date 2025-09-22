"""
SAM Protocol Test Suite
Comprehensive testing for protocol implementation
"""

import gc
import utime


# Test utilities
def test_crc16():
    """Test CRC-16-CCITT implementation"""
    print("\nTesting CRC-16-CCITT...")
    from utils.crc16_ccitt import calculate_crc16

    # Test vectors
    test_data = [
        (b"", 0xFFFF),
        (b"A", 0x9479),
        (b"123456789", 0x906E),
        (b"\x01\x02\x03\x04", 0x89C3),
    ]

    passed = 0
    for data, expected in test_data:
        calculated = calculate_crc16(data)
        if calculated == expected:
            print(f"  PASS: CRC({data}) = 0x{calculated:04X}")
            passed += 1
        else:
            print(
                f"  FAIL: CRC({data}) = 0x{calculated:04X}, expected 0x{expected:04X}"
            )

    print(f"CRC-16 Tests: {passed}/{len(test_data)} passed")
    return passed == len(test_data)


def test_circular_buffer():
    """Test circular buffer implementation"""
    print("\nTesting Circular Buffer...")
    from utils.circular_buffer import CircularBuffer

    buffer = CircularBuffer(10)
    tests_passed = 0

    # Test 1: Basic write/read
    buffer.put(b"Hello")
    result = buffer.get(5)
    if result == b"Hello":
        print("  PASS: Basic write/read")
        tests_passed += 1
    else:
        print(f"  FAIL: Basic write/read - got {result}")

    # Test 2: Overflow protection
    buffer.clear()
    added = buffer.put(b"0123456789ABC")  # 13 bytes into 10 byte buffer
    if added == 10 and buffer.available() == 10:
        print("  PASS: Overflow protection")
        tests_passed += 1
    else:
        print(f"  FAIL: Overflow - added {added}, available {buffer.available()}")

    # Test 3: Wrap around
    buffer.clear()
    buffer.put(b"01234")
    buffer.get(3)  # Remove 3 bytes
    buffer.put(b"56789")  # Add 5 more
    result = buffer.get()
    if result == b"2345678":
        print("  PASS: Wrap around")
        tests_passed += 1
    else:
        print(f"  FAIL: Wrap around - got {result}")

    print(f"Circular Buffer Tests: {tests_passed}/3 passed")
    return tests_passed == 3


def test_frame_encoding():
    """Test frame encoding with byte stuffing"""
    print("\nTesting Frame Encoding...")
    from protocol.sam_l2 import FrameEncoder
    from protocol.sam_defs import FLAG, ESC, ADDR_SAM, TYPE_INFO

    encoder = FrameEncoder()
    tests_passed = 0

    # Test 1: Simple frame
    frame = encoder.encode_frame(ADDR_SAM, TYPE_INFO, b"Test")
    if frame[0] == FLAG and frame[-1] == FLAG and len(frame) > 8:
        print("  PASS: Simple frame encoding")
        tests_passed += 1
    else:
        print(f"  FAIL: Simple frame - got {frame.hex()}")

    # Test 2: Frame with FLAG byte in payload
    payload = bytes([FLAG, 0x00, FLAG])
    frame = encoder.encode_frame(ADDR_SAM, TYPE_INFO, payload)
    # Should have escape sequences
    if ESC in frame[1:-1]:
        print("  PASS: Byte stuffing for FLAG")
        tests_passed += 1
    else:
        print(f"  FAIL: No byte stuffing - got {frame.hex()}")

    # Test 3: Frame with ESC byte in payload
    payload = bytes([ESC, 0x00, ESC])
    frame = encoder.encode_frame(ADDR_SAM, TYPE_INFO, payload)
    # Count ESC bytes (should be more than in payload due to stuffing)
    esc_count = sum(1 for b in frame[1:-1] if b == ESC)
    if esc_count > 2:
        print("  PASS: Byte stuffing for ESC")
        tests_passed += 1
    else:
        print(f"  FAIL: ESC stuffing - found {esc_count} ESC bytes")

    print(f"Frame Encoding Tests: {tests_passed}/3 passed")
    return tests_passed == 3


def test_frame_decoding():
    """Test frame decoding with byte unstuffing"""
    print("\nTesting Frame Decoding...")
    from protocol.sam_l2 import FrameEncoder, FrameDecoder
    from protocol.sam_defs import ADDR_SAM, TYPE_INFO

    encoder = FrameEncoder()
    received_frames = []

    def on_frame(addr, ctrl, payload):
        received_frames.append((addr, ctrl, payload))

    decoder = FrameDecoder(on_frame)
    tests_passed = 0

    # Test 1: Simple frame decode
    received_frames.clear()
    frame = encoder.encode_frame(ADDR_SAM, TYPE_INFO, b"Hello")
    for byte in frame:
        decoder.process_byte(byte)

    if len(received_frames) == 1 and received_frames[0][2] == b"Hello":
        print("  PASS: Simple frame decoding")
        tests_passed += 1
    else:
        print(f"  FAIL: Simple decode - got {received_frames}")

    # Test 2: Frame with special bytes
    received_frames.clear()
    payload = bytes([0x7E, 0x7D, 0x00])
    frame = encoder.encode_frame(ADDR_SAM, TYPE_INFO, payload)
    for byte in frame:
        decoder.process_byte(byte)

    if len(received_frames) == 1 and received_frames[0][2] == payload:
        print("  PASS: Special byte decoding")
        tests_passed += 1
    else:
        print(f"  FAIL: Special byte decode - got {received_frames}")

    # Test 3: CRC error detection
    received_frames.clear()
    frame = bytearray(encoder.encode_frame(ADDR_SAM, TYPE_INFO, b"Test"))
    frame[-3] ^= 0x01  # Corrupt CRC
    for byte in frame:
        decoder.process_byte(byte)

    if len(received_frames) == 0:
        print("  PASS: CRC error detection")
        tests_passed += 1
    else:
        print("  FAIL: CRC error not detected")

    print(f"Frame Decoding Tests: {tests_passed}/3 passed")
    return tests_passed == 3


def test_sliding_window():
    """Test sliding window protocol"""
    print("\nTesting Sliding Window...")
    from protocol.sam_transport import SlidingWindow

    window = SlidingWindow(4)
    tests_passed = 0

    # Test 1: Sequence number allocation
    seqs = []
    for i in range(4):
        seq = window.get_next_tx_seq()
        if seq is not None:
            seqs.append(seq)

    if seqs == [0, 1, 2, 3] and window.get_next_tx_seq() is None:
        print("  PASS: Window size enforcement")
        tests_passed += 1
    else:
        print(f"  FAIL: Window enforcement - got seqs {seqs}")

    # Test 2: ACK processing
    window.process_ack(2)  # ACK(2) acknowledges 0 and 1
    seq = window.get_next_tx_seq()
    if seq == 4:  # Should wrap to 4
        print("  PASS: ACK processing")
        tests_passed += 1
    else:
        print(f"  FAIL: ACK processing - got seq {seq}")

    # Test 3: Sequence wrap-around
    window.reset()
    window.tx_seq = 14
    seqs = []
    for i in range(4):
        seq = window.get_next_tx_seq()
        if seq is not None:
            seqs.append(seq)

    if seqs == [14, 15, 0, 1]:  # Should wrap at 16
        print("  PASS: Sequence wrap-around")
        tests_passed += 1
    else:
        print(f"  FAIL: Wrap-around - got {seqs}")

    print(f"Sliding Window Tests: {tests_passed}/3 passed")
    return tests_passed == 3


def test_priority_queue():
    """Test priority queue"""
    print("\nTesting Priority Queue...")
    from utils.priority_queue import MessagePriorityQueue

    queue = MessagePriorityQueue(5)
    tests_passed = 0

    # Test 1: Priority ordering
    queue.put_message(b"\x10\x00", None)  # Input, priority 1
    queue.put_message(b"\x30\x00", None)  # Power, priority 4
    queue.put_message(b"\x20\x00", None)  # Output, priority 2

    msg1 = queue.get()
    msg2 = queue.get()
    msg3 = queue.get()

    if msg1 == b"\x30\x00" and msg2 == b"\x20\x00" and msg3 == b"\x10\x00":
        print("  PASS: Priority ordering")
        tests_passed += 1
    else:
        print("  FAIL: Priority ordering")

    # Test 2: Queue overflow handling
    queue.clear()
    for i in range(6):
        added = queue.put_message(bytes([0x10, i]), None)

    if queue.size() == 5:
        print("  PASS: Queue size limit")
        tests_passed += 1
    else:
        print(f"  FAIL: Queue size {queue.size()}, expected 5")

    # Test 3: High priority preemption
    queue.clear()
    for i in range(5):
        queue.put_message(bytes([0x10, i]), None)  # Fill with low priority

    added = queue.put_message(b"\x30\x00", None)  # Try high priority
    if added and queue.peek() == b"\x30\x00":
        print("  PASS: High priority preemption")
        tests_passed += 1
    else:
        print("  FAIL: Preemption failed")

    print(f"Priority Queue Tests: {tests_passed}/3 passed")
    return tests_passed == 3


def test_full_stack():
    """Test full protocol stack integration"""
    print("\nTesting Full Protocol Stack...")
    from protocol.sam_transport import TransportLayer
    from protocol.sam_app import ApplicationLayer
    from drivers.uart_driver import LoopbackUART

    # Create loopback UART for testing
    uart = LoopbackUART()
    uart.start()

    # Create protocol stack
    transport = TransportLayer(uart)
    ApplicationLayer(transport)

    received_messages = []

    def on_message(msg):
        received_messages.append(msg)

    transport.on_message_received = on_message
    tests_passed = 0

    # Test 1: SYNC negotiation
    print("  Testing SYNC negotiation...")
    transport.start()

    # Simulate some processing cycles
    for _ in range(10):
        data = uart.read()
        if data:
            transport.process_rx_data(data)
        transport.process()
        utime.sleep_ms(10)

    # Check if SYNC was sent
    if transport.state == 1:  # STATE_SYNCING
        print("  PASS: SYNC initiated")
        tests_passed += 1
    else:
        print(f"  FAIL: State = {transport.state}")

    # Clean up
    transport.stop()
    uart.stop()

    print(f"Full Stack Tests: {tests_passed}/1 passed")
    return tests_passed == 1


def run_all_tests():
    """Run all protocol tests"""
    print("=" * 50)
    print("SAM Protocol v2.0.0 Test Suite")
    print("=" * 50)

    total_tests = 0
    passed_tests = 0

    # Run each test module
    tests = [
        ("CRC-16", test_crc16),
        ("Circular Buffer", test_circular_buffer),
        ("Frame Encoding", test_frame_encoding),
        ("Frame Decoding", test_frame_decoding),
        ("Sliding Window", test_sliding_window),
        ("Priority Queue", test_priority_queue),
        ("Full Stack", test_full_stack),
    ]

    for name, test_func in tests:
        try:
            gc.collect()
            if test_func():
                passed_tests += 1
            total_tests += 1
        except Exception as e:
            print(f"\nERROR in {name}: {e}")
            total_tests += 1

    # Summary
    print("\n" + "=" * 50)
    print(f"Test Results: {passed_tests}/{total_tests} modules passed")
    if passed_tests == total_tests:
        print("ALL TESTS PASSED!")
    else:
        print(f"FAILURES: {total_tests - passed_tests} modules failed")
    print("=" * 50)

    return passed_tests == total_tests


if __name__ == "__main__":
    # Run tests
    success = run_all_tests()

    # Show memory usage
    gc.collect()
    print(f"\nMemory: {gc.mem_free()} bytes free")

    # Exit with status
    import sys

    sys.exit(0 if success else 1)
