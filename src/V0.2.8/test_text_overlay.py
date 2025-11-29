#!/usr/bin/env python3
"""
Test script for text_overlay module.

This script tests the text overlay functionality on your Mac before deploying to RP2040.
It loads a base image, applies text overlays, and saves the result for visual inspection.

Usage:
    python3 test_text_overlay.py

Output:
    asset/test_output.png - Result image for inspection

Author: PamirAI
"""

import sys
from pathlib import Path

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from text_overlay import TextOverlay, draw_error, FONT_WIDTH, FONT_HEIGHT
from bin_viewer import bin_to_image, buffer_to_image, bin_to_pil, pil_to_bin

# Paths
SCRIPT_DIR = Path(__file__).parent
BIN_DIR = SCRIPT_DIR / "bin"
ASSET_DIR = SCRIPT_DIR / "asset"


def test_basic_text():
    """Test basic text rendering."""
    print("Test 1: Basic text rendering")

    # Load base image
    base_bin = BIN_DIR / "loading1.bin"
    if not base_bin.exists():
        print(f"  Error: {base_bin} not found. Run generate_boot_images.py first.")
        return False

    with open(base_bin, 'rb') as f:
        buffer = bytearray(f.read())

    overlay = TextOverlay()

    # Draw some test text
    overlay.draw_text(buffer, "Hello World", 10, 10, color=0)  # black text
    overlay.draw_text(buffer, "TEST 123", 10, 25, color=0)

    # Save result
    output_path = ASSET_DIR / "test_basic.png"
    buffer_to_image(buffer, str(output_path))
    print(f"  Saved: {output_path}")
    return True


def test_text_box():
    """Test text box with background."""
    print("Test 2: Text box with background")

    base_bin = BIN_DIR / "loading1.bin"
    with open(base_bin, 'rb') as f:
        buffer = bytearray(f.read())

    overlay = TextOverlay()

    # Draw text boxes
    overlay.draw_text_box(buffer, "ERR:0x1A", 5, 5, color=1, bg_color=0, padding=2)
    overlay.draw_text_box(buffer, "Status: OK", 5, 25, color=0, bg_color=1, padding=1)
    overlay.draw_text_box(buffer, "v0.2.8", 5, 45, color=1, bg_color=0, padding=1)

    output_path = ASSET_DIR / "test_textbox.png"
    buffer_to_image(buffer, str(output_path))
    print(f"  Saved: {output_path}")
    return True


def test_error_display():
    """Test error display helper."""
    print("Test 3: Error display helper")

    base_bin = BIN_DIR / "error.bin"
    with open(base_bin, 'rb') as f:
        buffer = bytearray(f.read())

    # Use convenience function
    draw_error(buffer, 0x1A, x=20, y=100)

    output_path = ASSET_DIR / "test_error.png"
    buffer_to_image(buffer, str(output_path))
    print(f"  Saved: {output_path}")
    return True


def test_all_characters():
    """Test all font characters."""
    print("Test 4: All font characters")

    # Create blank white buffer
    buffer = bytearray([0xFF] * 4000)

    overlay = TextOverlay()

    # Draw all characters
    chars_line1 = "0123456789"
    chars_line2 = "ABCDEFGHIJ"
    chars_line3 = "KLMNOPQRST"
    chars_line4 = "UVWXYZ"
    chars_line5 = "abcdefghij"
    chars_line6 = "klmnopqrst"
    chars_line7 = "uvwxyz"
    chars_line8 = " :-_./()"
    chars_line9 = "[]!?#%=+*<>"

    y = 5
    for line in [chars_line1, chars_line2, chars_line3, chars_line4,
                 chars_line5, chars_line6, chars_line7, chars_line8, chars_line9]:
        overlay.draw_text(buffer, line, 5, y, color=0)
        y += FONT_HEIGHT + 2

    output_path = ASSET_DIR / "test_font.png"
    buffer_to_image(buffer, str(output_path))
    print(f"  Saved: {output_path}")
    return True


def test_positioning():
    """Test various positions on the display."""
    print("Test 5: Position testing")

    base_bin = BIN_DIR / "loading1.bin"
    with open(base_bin, 'rb') as f:
        buffer = bytearray(f.read())

    overlay = TextOverlay()

    # Draw at various positions to test boundaries
    overlay.draw_text_box(buffer, "TOP-LEFT", 0, 0, color=1, bg_color=0)
    overlay.draw_text_box(buffer, "MIDDLE", 40, 120, color=1, bg_color=0)
    overlay.draw_text_box(buffer, "BOTTOM", 5, 235, color=1, bg_color=0)

    # Draw position markers
    overlay.draw_text(buffer, "X:0", 0, 50, color=0)
    overlay.draw_text(buffer, "X:64", 64, 50, color=0)
    overlay.draw_text(buffer, "Y:100", 5, 100, color=0)
    overlay.draw_text(buffer, "Y:200", 5, 200, color=0)

    output_path = ASSET_DIR / "test_position.png"
    buffer_to_image(buffer, str(output_path))
    print(f"  Saved: {output_path}")
    return True


def test_on_blank():
    """Test on blank canvas to clearly see font rendering."""
    print("Test 6: Blank canvas test")

    # Create blank white buffer
    buffer = bytearray([0xFF] * 4000)

    overlay = TextOverlay()

    # Draw various text
    overlay.draw_text_box(buffer, "ERROR: 0x1A", 5, 5, color=1, bg_color=0, padding=2)
    overlay.draw_text(buffer, "Normal black text", 5, 30, color=0)
    overlay.draw_text_box(buffer, "White on Black", 5, 50, color=1, bg_color=0)
    overlay.draw_text_box(buffer, "Black on White", 5, 70, color=0, bg_color=1)

    # Show font info
    overlay.draw_text(buffer, "Font: 6x8 TinyUnicode", 5, 100, color=0)
    overlay.draw_text(buffer, "Display: 128x250", 5, 115, color=0)

    output_path = ASSET_DIR / "test_blank.png"
    buffer_to_image(buffer, str(output_path))
    print(f"  Saved: {output_path}")
    return True


def main():
    """Run all tests."""
    print("=" * 50)
    print("Text Overlay Test Suite")
    print("=" * 50)
    print()

    # Ensure asset directory exists
    ASSET_DIR.mkdir(exist_ok=True)

    # Check for base binary
    if not (BIN_DIR / "loading1.bin").exists():
        print("Warning: loading1.bin not found in bin/")
        print("Running generate_boot_images.py first...")
        import subprocess
        subprocess.run([sys.executable, "generate_boot_images.py"], cwd=SCRIPT_DIR)
        print()

    # Run tests
    tests = [
        test_basic_text,
        test_text_box,
        test_error_display,
    ]

    passed = 0
    failed = 0

    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  FAILED: {e}")
            failed += 1
        print()

    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    print(f"Check output images in: {ASSET_DIR}")
    print("=" * 50)


if __name__ == "__main__":
    main()
