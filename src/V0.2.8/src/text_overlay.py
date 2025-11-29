"""
Text Overlay - Lightweight text rendering for e-ink display.

This module renders text directly onto packed byte arrays for the e-ink display.
Designed to run on both MicroPython (RP2040) and standard Python 3 (for testing).

No external dependencies - uses hardcoded bitmap font extracted from TinyUnicode.ttf.

E-ink display format:
- 128x250 pixels, 1-bit (black/white)
- Packed as 4000 bytes (128 * 250 / 8)
- Each byte = 8 horizontal pixels, MSB = leftmost pixel
- 0 = black, 1 = white

Author: PamirAI
"""

# Display constants
EPD_WIDTH = 128
EPD_HEIGHT = 250
BYTES_PER_ROW = EPD_WIDTH // 8  # 16 bytes per row

# Font constants - extracted from TinyUnicode.ttf at size 16
FONT_WIDTH = 6   # pixels per character (includes 1px spacing)
FONT_HEIGHT = 8  # pixels per character (includes 1px spacing)

# Font bitmap data - each character is 8 bytes (one per row), MSB-aligned
# Extracted from TinyUnicode.ttf
FONT_DATA = {
    '0': (0x60, 0x90, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00),
    '1': (0x40, 0xC0, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00),
    '2': (0x60, 0x90, 0x20, 0x40, 0xF0, 0x00, 0x00, 0x00),
    '3': (0xE0, 0x10, 0x70, 0x10, 0xE0, 0x00, 0x00, 0x00),
    '4': (0x20, 0x60, 0xA0, 0xF0, 0x20, 0x00, 0x00, 0x00),
    '5': (0xF0, 0x80, 0xE0, 0x10, 0xE0, 0x00, 0x00, 0x00),
    '6': (0x60, 0x80, 0xE0, 0x90, 0x60, 0x00, 0x00, 0x00),
    '7': (0xF0, 0x10, 0x20, 0x40, 0x40, 0x00, 0x00, 0x00),
    '8': (0x60, 0x90, 0x60, 0x90, 0x60, 0x00, 0x00, 0x00),
    '9': (0x60, 0x90, 0x70, 0x10, 0x60, 0x00, 0x00, 0x00),
    'A': (0x60, 0x90, 0xF0, 0x90, 0x90, 0x00, 0x00, 0x00),
    'B': (0xE0, 0x90, 0xE0, 0x90, 0xE0, 0x00, 0x00, 0x00),
    'C': (0x60, 0x80, 0x80, 0x80, 0x60, 0x00, 0x00, 0x00),
    'D': (0xE0, 0x90, 0x90, 0x90, 0xE0, 0x00, 0x00, 0x00),
    'E': (0xE0, 0x80, 0xC0, 0x80, 0xE0, 0x00, 0x00, 0x00),
    'F': (0xE0, 0x80, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00),
    'G': (0x70, 0x80, 0xB0, 0x90, 0x70, 0x00, 0x00, 0x00),
    'H': (0x90, 0x90, 0xF0, 0x90, 0x90, 0x00, 0x00, 0x00),
    'I': (0xE0, 0x40, 0x40, 0x40, 0xE0, 0x00, 0x00, 0x00),
    'J': (0x70, 0x10, 0x10, 0x90, 0x60, 0x00, 0x00, 0x00),
    'K': (0x90, 0xA0, 0xC0, 0xA0, 0x90, 0x00, 0x00, 0x00),
    'L': (0x80, 0x80, 0x80, 0x80, 0xE0, 0x00, 0x00, 0x00),
    'M': (0x88, 0xD8, 0xA8, 0x88, 0x88, 0x00, 0x00, 0x00),
    'N': (0x90, 0xD0, 0xB0, 0x90, 0x90, 0x00, 0x00, 0x00),
    'O': (0x60, 0x90, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00),
    'P': (0xE0, 0x90, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00),
    'Q': (0x60, 0x90, 0x90, 0x90, 0x60, 0x10, 0x00, 0x00),
    'R': (0xE0, 0x90, 0x90, 0xE0, 0x90, 0x00, 0x00, 0x00),
    'S': (0x70, 0x80, 0x60, 0x10, 0xE0, 0x00, 0x00, 0x00),
    'T': (0xE0, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00),
    'U': (0x90, 0x90, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00),
    'V': (0x90, 0x90, 0xA0, 0xA0, 0x40, 0x00, 0x00, 0x00),
    'W': (0x88, 0x88, 0xA8, 0xA8, 0x50, 0x00, 0x00, 0x00),
    'X': (0x90, 0x90, 0x60, 0x90, 0x90, 0x00, 0x00, 0x00),
    'Y': (0x90, 0x90, 0x70, 0x10, 0x60, 0x00, 0x00, 0x00),
    'Z': (0xE0, 0x20, 0x40, 0x80, 0xE0, 0x00, 0x00, 0x00),
    'a': (0x70, 0x90, 0x90, 0x70, 0x00, 0x00, 0x00, 0x00),
    'b': (0x80, 0xE0, 0x90, 0x90, 0xE0, 0x00, 0x00, 0x00),
    'c': (0x60, 0x80, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00),
    'd': (0x10, 0x70, 0x90, 0x90, 0x70, 0x00, 0x00, 0x00),
    'e': (0x60, 0xB0, 0xC0, 0x60, 0x00, 0x00, 0x00, 0x00),
    'f': (0x60, 0x40, 0xE0, 0x40, 0x40, 0x00, 0x00, 0x00),
    'g': (0x70, 0x90, 0x90, 0x70, 0x10, 0x60, 0x00, 0x00),
    'h': (0x80, 0xE0, 0x90, 0x90, 0x90, 0x00, 0x00, 0x00),
    'i': (0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00),
    'j': (0x40, 0x00, 0x40, 0x40, 0x40, 0x40, 0x80, 0x00),
    'k': (0x80, 0x90, 0xA0, 0xE0, 0x90, 0x00, 0x00, 0x00),
    'l': (0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00),
    'm': (0xF0, 0xA8, 0xA8, 0xA8, 0x00, 0x00, 0x00, 0x00),
    'n': (0xE0, 0x90, 0x90, 0x90, 0x00, 0x00, 0x00, 0x00),
    'o': (0x60, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00, 0x00),
    'p': (0xE0, 0x90, 0x90, 0xE0, 0x80, 0x80, 0x00, 0x00),
    'q': (0x70, 0x90, 0x90, 0x70, 0x10, 0x10, 0x00, 0x00),
    'r': (0xA0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00),
    's': (0x70, 0xC0, 0x30, 0xE0, 0x00, 0x00, 0x00, 0x00),
    't': (0x40, 0xE0, 0x40, 0x40, 0x60, 0x00, 0x00, 0x00),
    'u': (0x90, 0x90, 0x90, 0x70, 0x00, 0x00, 0x00, 0x00),
    'v': (0x90, 0x90, 0xA0, 0x40, 0x00, 0x00, 0x00, 0x00),
    'w': (0x88, 0xA8, 0xA8, 0x50, 0x00, 0x00, 0x00, 0x00),
    'x': (0xA0, 0x40, 0x40, 0xA0, 0x00, 0x00, 0x00, 0x00),
    'y': (0x90, 0x90, 0x90, 0x70, 0x10, 0xE0, 0x00, 0x00),
    'z': (0xF0, 0x20, 0x40, 0xF0, 0x00, 0x00, 0x00, 0x00),
    ' ': (0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    ':': (0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00),
    '-': (0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    '_': (0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00),
    '.': (0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    '/': (0x20, 0x20, 0x40, 0x80, 0x80, 0x00, 0x00, 0x00),
    '(': (0x20, 0x40, 0x40, 0x40, 0x40, 0x20, 0x00, 0x00),
    ')': (0x80, 0x40, 0x40, 0x40, 0x40, 0x80, 0x00, 0x00),
    '[': (0x60, 0x40, 0x40, 0x40, 0x40, 0x60, 0x00, 0x00),
    ']': (0xC0, 0x40, 0x40, 0x40, 0x40, 0xC0, 0x00, 0x00),
    '!': (0x80, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00),
    '?': (0x60, 0x90, 0x20, 0x00, 0x40, 0x00, 0x00, 0x00),
    '#': (0x50, 0xF8, 0x50, 0xF8, 0x50, 0x00, 0x00, 0x00),
    '%': (0x88, 0x10, 0x20, 0x40, 0x88, 0x00, 0x00, 0x00),
    '=': (0xF0, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00),
    '+': (0x40, 0xE0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00),
    '*': (0xA0, 0x40, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00),
    '<': (0x20, 0x40, 0x80, 0x40, 0x20, 0x00, 0x00, 0x00),
    '>': (0x80, 0x40, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00),
}


class TextOverlay:
    """
    Lightweight text overlay for e-ink display.

    Renders text directly onto packed byte arrays.
    Works on both MicroPython and standard Python.
    """

    def __init__(self, width=EPD_WIDTH, height=EPD_HEIGHT):
        """
        Initialize TextOverlay.

        Args:
            width: Display width in pixels (default 128)
            height: Display height in pixels (default 250)
        """
        self.width = width
        self.height = height
        self.bytes_per_row = width // 8

    def _set_pixel(self, buffer, x, y, color):
        """
        Set a single pixel in the packed buffer.

        Args:
            buffer: bytearray of packed pixel data
            x: x coordinate (0 = left)
            y: y coordinate (0 = top)
            color: 0 = black, 1 = white
        """
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return  # Out of bounds

        byte_idx = y * self.bytes_per_row + (x >> 3)  # x // 8
        bit_pos = 7 - (x & 7)  # 7 - (x % 8)

        if color:
            buffer[byte_idx] |= (1 << bit_pos)   # Set bit (white)
        else:
            buffer[byte_idx] &= ~(1 << bit_pos)  # Clear bit (black)

    def _fill_rect(self, buffer, x, y, w, h, color):
        """
        Fill a rectangle with a color.

        Args:
            buffer: bytearray of packed pixel data
            x, y: top-left corner
            w, h: width and height
            color: 0 = black, 1 = white
        """
        for py in range(y, y + h):
            for px in range(x, x + w):
                self._set_pixel(buffer, px, py, color)

    def draw_char(self, buffer, char, x, y, color=0, rotation=90):
        """
        Draw a single character at position (x, y).

        Args:
            buffer: bytearray of packed pixel data
            char: character to draw
            x: x position (left edge of character)
            y: y position (top edge of character)
            color: 0 = black, 1 = white
            rotation: 0, 90, 180, or 270 degrees CCW (default 90)

        Returns:
            Tuple of (char_width, char_height) after rotation
        """
        if char not in FONT_DATA:
            char = '?'  # Unknown character fallback

        glyph = FONT_DATA[char]

        # Draw each row of the glyph
        for row_idx, row_byte in enumerate(glyph):
            # Check each bit in the row (only first FONT_WIDTH bits used)
            for bit in range(FONT_WIDTH):
                # Check if this pixel is set in the glyph (MSB = leftmost)
                if row_byte & (1 << (7 - bit)):
                    # Calculate rotated position
                    if rotation == 0:
                        px = x + bit
                        py = y + row_idx
                    elif rotation == 90:
                        # 90° CCW: flip vertically by inverting row_idx
                        px = x + (FONT_HEIGHT - 1 - row_idx)
                        py = y + (FONT_WIDTH - 1 - bit)
                    elif rotation == 180:
                        px = x + (FONT_WIDTH - 1 - bit)
                        py = y + (FONT_HEIGHT - 1 - row_idx)
                    elif rotation == 270:
                        px = x + row_idx
                        py = y + bit
                    else:
                        px = x + bit
                        py = y + row_idx

                    self._set_pixel(buffer, px, py, color)

        # Return dimensions after rotation
        if rotation in (90, 270):
            return (FONT_HEIGHT, FONT_WIDTH)  # width, height swapped
        return (FONT_WIDTH, FONT_HEIGHT)

    def draw_text(self, buffer, text, x, y, color=0, spacing=0, rotation=90):
        """
        Draw a string of text at position (x, y).

        Args:
            buffer: bytearray of packed pixel data
            text: string to draw
            x: x position (left edge)
            y: y position (top edge)
            color: 0 = black, 1 = white
            spacing: extra spacing between characters (default 0)
            rotation: 0, 90, 180, or 270 degrees CCW (default 90)

        Returns:
            Tuple of (total_width, total_height) of text drawn
        """
        cursor_x = x
        cursor_y = y

        # For 90° and 180° rotation, reverse the text so it reads correctly
        # Use join+reversed instead of [::-1] for MicroPython compatibility
        if rotation in (90, 180):
            text = ''.join(reversed(text))

        for char in text:
            char_dims = self.draw_char(buffer, char, cursor_x, cursor_y, color, rotation)
            char_w, char_h = char_dims

            # Advance cursor based on rotation
            if rotation == 0:
                cursor_x += char_w + spacing  # Move right
            elif rotation == 90:
                cursor_y += char_h + spacing  # Move down
            elif rotation == 180:
                cursor_x -= char_w + spacing  # Move left
            elif rotation == 270:
                cursor_y -= char_h + spacing  # Move up

        # Return total dimensions
        if rotation == 0:
            return (cursor_x - x, FONT_HEIGHT)
        elif rotation == 90:
            return (FONT_HEIGHT, cursor_y - y)
        elif rotation == 180:
            return (x - cursor_x, FONT_HEIGHT)
        elif rotation == 270:
            return (FONT_HEIGHT, y - cursor_y)
        return (cursor_x - x, FONT_HEIGHT)

    def draw_text_box(self, buffer, text, x, y, color=0, bg_color=1, padding=1, rotation=90):
        """
        Draw text with a background box.

        Args:
            buffer: bytearray of packed pixel data
            text: string to draw
            x: x position (left edge of box)
            y: y position (top edge of box)
            color: text color (0 = black, 1 = white)
            bg_color: background color (0 = black, 1 = white)
            padding: padding around text in pixels
            rotation: 0, 90, 180, or 270 degrees CCW (default 90)

        Returns:
            Tuple of (box_width, box_height)
        """
        # Calculate text dimensions based on rotation
        text_len = len(text)
        if rotation in (90, 270):
            # Text is vertical
            text_width = FONT_HEIGHT
            text_height = text_len * FONT_WIDTH
        else:
            # Text is horizontal
            text_width = text_len * FONT_WIDTH
            text_height = FONT_HEIGHT

        box_width = text_width + padding * 2
        box_height = text_height + padding * 2

        # Draw background
        self._fill_rect(buffer, x, y, box_width, box_height, bg_color)

        # Draw text at padded position
        self.draw_text(buffer, text, x + padding, y + padding, color, rotation=rotation)

        return (box_width, box_height)

    def get_text_width(self, text, rotation=90):
        """
        Get the pixel width of a text string.

        Args:
            text: string to measure
            rotation: 0, 90, 180, or 270 degrees CCW (default 90)

        Returns:
            Width in pixels
        """
        if rotation in (90, 270):
            return FONT_HEIGHT
        return len(text) * FONT_WIDTH

    def get_text_height(self, text="", rotation=90):
        """
        Get the pixel height of text.

        Args:
            text: string to measure (needed for vertical text)
            rotation: 0, 90, 180, or 270 degrees CCW (default 90)

        Returns:
            Height in pixels
        """
        if rotation in (90, 270):
            return len(text) * FONT_WIDTH if text else FONT_WIDTH
        return FONT_HEIGHT

    def get_text_dimensions(self, text, rotation=90):
        """
        Get the pixel dimensions (width, height) of text with rotation.

        Args:
            text: string to measure
            rotation: 0, 90, 180, or 270 degrees CCW (default 90)

        Returns:
            Tuple of (width, height) in pixels
        """
        text_len = len(text)
        if rotation in (90, 270):
            return (FONT_HEIGHT, text_len * FONT_WIDTH)
        return (text_len * FONT_WIDTH, FONT_HEIGHT)


# Convenience function for simple usage
def draw_error(buffer, error_code, message="", x=2, y=2, rotation=90):
    """
    Draw an error message on the display.

    Args:
        buffer: bytearray of packed pixel data
        error_code: integer error code (will be displayed as hex)
        message: optional error message string
        x, y: position for error box
        rotation: 0, 90, 180, or 270 degrees CCW (default 90)
    """
    overlay = TextOverlay()

    if message:
        text = "ERR:0x{:02X} {}".format(error_code, message)
    else:
        text = "ERR:0x{:02X}".format(error_code)

    overlay.draw_text_box(buffer, text, x, y, color=1, bg_color=0, padding=2, rotation=rotation)

