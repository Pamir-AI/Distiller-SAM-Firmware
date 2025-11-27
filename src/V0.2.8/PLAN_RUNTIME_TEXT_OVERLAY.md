# Plan: Runtime Text Overlay for RP2040 (MicroPython)

## Overview
Create a lightweight text overlay module for the RP2040 (SAM) that can render text directly onto the e-ink display's byte array buffer at runtime. This is for displaying error codes and status messages without needing PIL or external image processing.

## Development Strategy

### Two-Phase Approach

**Phase 1: Develop and test on Mac (Python 3 + PIL for visualization)**
- Write the core text overlay logic in pure Python (no MicroPython-specific code)
- Create a visualization helper that converts byte array → PNG for viewing
- Iterate on font design, positioning, and rendering until correct
- All testing done on Mac before deploying to RP2040

**Phase 2: Deploy to RP2040**
- Copy the tested `text_overlay.py` to the device
- Remove any debug/visualization code (or guard with `try/except ImportError`)
- The core logic remains identical

### Visualization Helper (Mac only)

Create `bin_viewer.py` in V0.2.8/ that:
1. Loads a .bin file (4000 bytes)
2. Unpacks to pixel array
3. Converts to PNG using PIL
4. Saves/displays the image

This lets you:
- See the raw byte stream as an image
- Test text overlay positioning without flashing to device
- Fine-tune bounding boxes and coordinates
- Verify font rendering is correct

## Analysis

### Current Architecture
- E-ink display: 128x250 pixels, 1-bit (black/white)
- Data format: Packed byte array, 4000 bytes (128 * 250 / 8)
- Each byte = 8 horizontal pixels, MSB = leftmost pixel
- Driver methods: `EPD_Display(image)`, `epd_display_part_all(image)` take byte arrays

### Key Decision: When to Overlay?

**Option A: Modify byte array before sending to display**
- Pros:
  - Work directly on the packed binary format
  - Can modify existing image buffer in-place
  - Memory efficient (no intermediate format)
- Cons:
  - Bit manipulation is more complex
  - Need to handle byte boundaries for character placement

**Option B: Create unpacked framebuffer, draw, then pack**
- Pros:
  - Simpler pixel-level manipulation
  - Easier to implement character rendering
- Cons:
  - Requires 32KB RAM for unpacked buffer (128*250 bytes) - TOO MUCH for RP2040
  - Extra packing step

**Decision: Option A** - Work directly on the packed byte array. The RP2040 has limited RAM (~264KB total, much less available), so we cannot afford a full unpacked framebuffer.

### Font Strategy

**Option 1: Hardcoded bitmap font**
- Store a minimal character set as packed bit patterns
- Characters: 0-9, A-Z, a-z, space, colon, period, dash, underscore
- Fixed width (e.g., 5x7 or 6x8 pixels) for simplicity
- Stored as const bytes in code (no file I/O)

**Option 2: Load font from file**
- More flexible but requires file I/O
- Uses flash storage

**Decision: Option 1** - Hardcoded bitmap font. Minimal resource usage, no file I/O overhead at runtime.

## Design

### Module: `text_overlay.py`

```
src/
└── text_overlay.py   # New module
```

### Font Data Structure

Font extracted from TinyUnicode.ttf at size 16. Each character is 6 pixels wide x 8 pixels tall (includes spacing).
Each byte represents one row, MSB-aligned (bit 7 = leftmost pixel).

```python
FONT_WIDTH = 6
FONT_HEIGHT = 8

# Example visualization of 'A':
#   .██.....  = 0x60
#   █..█....  = 0x90
#   ████....  = 0xF0
#   █..█....  = 0x90
#   █..█....  = 0x90
#   ........  = 0x00
#   ........  = 0x00
#   ........  = 0x00

FONT_DATA = {
    '0': (0x60, 0x90, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00),
    '1': (0x40, 0xC0, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00),
    '2': (0x60, 0x90, 0x20, 0x40, 0xF0, 0x00, 0x00, 0x00),
    # ... 68 characters total: 0-9, A-Z, a-z, space, :, -, _, ., /
}
```

### Core Functions

```python
class TextOverlay:
    """Lightweight text overlay for e-ink display on RP2040"""

    def __init__(self, width=128, height=250):
        self.width = width
        self.height = height
        self.bytes_per_row = width // 8

    def draw_char(self, buffer, char, x, y, color=0):
        """
        Draw a single character onto the packed byte buffer.

        Args:
            buffer: bytearray of packed pixel data (4000 bytes)
            char: single character to draw
            x: x position (pixel)
            y: y position (pixel)
            color: 0=black, 1=white
        """
        pass

    def draw_text(self, buffer, text, x, y, color=0):
        """
        Draw a text string onto the packed byte buffer.

        Args:
            buffer: bytearray of packed pixel data
            text: string to draw
            x: starting x position
            y: starting y position
            color: 0=black, 1=white
        """
        pass

    def draw_text_box(self, buffer, text, x, y, color=0, bg_color=1):
        """
        Draw text with a background box.

        Args:
            buffer: bytearray of packed pixel data
            text: string to draw
            x, y: position
            color: text color (0=black, 1=white)
            bg_color: background color
        """
        pass
```

### Bit Manipulation Details

For a 128-wide display with 8 pixels per byte:
- Pixel at (x, y) is in byte at index: `y * 16 + x // 8`
- Bit position within byte: `7 - (x % 8)` (MSB = leftmost)

To set a pixel:
```python
byte_idx = y * bytes_per_row + x // 8
bit_pos = 7 - (x % 8)
if color:  # white
    buffer[byte_idx] |= (1 << bit_pos)
else:  # black
    buffer[byte_idx] &= ~(1 << bit_pos)
```

### Character Rendering (Rotated 90° CCW)

Since text needs to be rotated 90° counter-clockwise to match the display orientation:
- Original char: 5 wide x 7 tall
- After rotation: 7 wide x 5 tall
- The font data should be pre-rotated, OR we rotate during rendering

**Pre-rotated font is more efficient** - do the rotation once when defining the font, not at runtime.

### Memory Considerations

- Font data: ~70 characters × 7 bytes = ~490 bytes (stored in flash as const)
- Working buffer: 0 bytes (modify in-place)
- Code size: ~2-3KB estimated

### Usage Example

```python
from text_overlay import TextOverlay

# Load existing image
with open('loading1.bin', 'rb') as f:
    buffer = bytearray(f.read())

# Create overlay instance
overlay = TextOverlay()

# Draw error message
overlay.draw_text_box(buffer, "ERR:0x1A", x=5, y=200, color=1, bg_color=0)

# Display
eink.EPD_Display(buffer)
```

## Implementation Steps

1. **Create `text_overlay.py` module**
   - Define font data (5x7 bitmap, pre-rotated 90° CCW)
   - Character set: 0-9, A-Z, a-z, space, colon, period, dash, underscore, slash

2. **Implement `TextOverlay` class**
   - `__init__`: Store display dimensions
   - `_set_pixel`: Helper to set single pixel in packed buffer
   - `_fill_rect`: Helper to fill rectangle (for background)
   - `draw_char`: Render single character
   - `draw_text`: Render string of characters
   - `draw_text_box`: Render text with background box

3. **Optimize for speed**
   - Use `const()` for font data
   - Minimize function calls in inner loops
   - Consider using `@micropython.native` or `@micropython.viper` decorators

4. **Test on device**
   - Verify correct pixel placement
   - Check memory usage
   - Measure render time

## Font Design (5x7, will be stored pre-rotated as 7x5)

Using a standard 5x7 bitmap font. After 90° CCW rotation:
- Width becomes 7 pixels
- Height becomes 5 pixels
- Rows become columns (reading bottom-to-top becomes left-to-right)

## API Summary

```python
# Simple usage
overlay = TextOverlay()
overlay.draw_text(buffer, "Hello", 10, 100)

# With background box
overlay.draw_text_box(buffer, "ERR:42", 5, 200, color=1, bg_color=0)

# Error display helper
overlay.draw_error(buffer, error_code=0x1A, message="UART Timeout")
```

## Notes

- All rendering is done in-place on the byte buffer
- No PIL, no external dependencies on RP2040
- Font is compiled into the code (no file I/O)
- Rotation is pre-baked into font data
- Minimal RAM usage (~500 bytes for font in flash)

---

## File Structure

```
V0.2.8/
├── bin_viewer.py          # NEW: Mac-only visualization tool
├── text_overlay.py        # NEW: Core overlay logic (runs on both Mac and RP2040)
├── test_text_overlay.py   # NEW: Test script for Mac development
├── bin/
│   ├── loading1.bin
│   └── loading2.bin
├── asset/
│   └── (test output images)
└── src/
    ├── text_overlay.py    # COPY: Same file deployed to RP2040
    └── (other RP2040 modules)
```

## Implementation Order

### Step 1: Create `bin_viewer.py` (Mac only)
Visualization helper to convert .bin → PNG

```python
def bin_to_image(bin_path, output_path=None):
    """Convert packed binary to viewable PNG"""

def image_to_bin(image_path, output_path):
    """Convert PNG back to binary (for testing)"""
```

### Step 2: Create `text_overlay.py` (Cross-platform)
Core text overlay logic - pure Python, no PIL dependency

```python
class TextOverlay:
    def draw_text(self, buffer, text, x, y, color=0)
    def draw_text_box(self, buffer, text, x, y, color=0, bg_color=1)
```

### Step 3: Create `test_text_overlay.py` (Mac only)
Test harness that:
1. Loads loading1.bin
2. Applies text overlay
3. Saves result using bin_viewer
4. You visually inspect the output PNG

### Step 4: Iterate and fine-tune
- Adjust font data if needed
- Test different positions
- Verify text doesn't overflow

### Step 5: Deploy to RP2040
- Copy `text_overlay.py` to `src/`
- Update `upload.py` PYTHON_FILES list to include it
- Test on actual device

## Development Workflow

```bash
# On Mac - edit and test
python3 test_text_overlay.py

# View result
open asset/test_output.png

# When satisfied, deploy
python3 upload.py --generate
```
