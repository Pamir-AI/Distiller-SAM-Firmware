#!/usr/bin/env python3
"""
Binary Viewer - Visualization helper for e-ink display binary files.

Converts between packed binary format (.bin) and PNG images for viewing/debugging.
This is a Mac-only tool that uses PIL for image handling.

E-ink display format:
- 128x250 pixels, 1-bit (black/white)
- Packed as 4000 bytes (128 * 250 / 8)
- Each byte = 8 horizontal pixels, MSB = leftmost pixel
- 0 = black, 1 = white

Author: PamirAI
"""

import sys
from pathlib import Path

try:
    from PIL import Image
    import numpy as np
except ImportError:
    print("This tool requires PIL and numpy. Install with: pip install Pillow numpy")
    sys.exit(1)


# Display constants
EPD_WIDTH = 128
EPD_HEIGHT = 250
EPD_ARRAY = EPD_WIDTH * EPD_HEIGHT // 8  # 4000 bytes


def bin_to_image(bin_path: str, output_path: str = None, show: bool = False) -> Image.Image:
    """
    Convert packed binary file to PNG image.

    Args:
        bin_path: Path to .bin file (4000 bytes)
        output_path: Optional path to save PNG (if None, returns image without saving)
        show: If True, display the image

    Returns:
        PIL Image object
    """
    bin_path = Path(bin_path)

    if not bin_path.exists():
        raise FileNotFoundError(f"Binary file not found: {bin_path}")

    # Read binary data
    with open(bin_path, 'rb') as f:
        data = f.read()

    if len(data) != EPD_ARRAY:
        raise ValueError(f"Invalid file size: {len(data)} bytes (expected {EPD_ARRAY})")

    # Convert to image
    image = bin_to_pil(data)

    # Save if output path provided
    if output_path:
        image.save(output_path)
        print(f"Saved: {output_path}")

    # Show if requested
    if show:
        image.show()

    return image


def bin_to_pil(data: bytes) -> Image.Image:
    """
    Convert packed binary data to PIL Image.

    Args:
        data: Packed binary data (4000 bytes)

    Returns:
        PIL Image object (mode 'L', grayscale)
    """
    if len(data) != EPD_ARRAY:
        raise ValueError(f"Invalid data size: {len(data)} bytes (expected {EPD_ARRAY})")

    # Unpack bits to pixel array
    pixels = np.zeros((EPD_HEIGHT, EPD_WIDTH), dtype=np.uint8)

    for y in range(EPD_HEIGHT):
        for x_byte in range(EPD_WIDTH // 8):
            byte_idx = y * (EPD_WIDTH // 8) + x_byte
            byte_val = data[byte_idx]

            for bit in range(8):
                x = x_byte * 8 + bit
                # MSB = leftmost pixel, 1 = white, 0 = black
                if byte_val & (1 << (7 - bit)):
                    pixels[y, x] = 255  # white
                else:
                    pixels[y, x] = 0    # black

    return Image.fromarray(pixels, mode='L')


def pil_to_bin(image: Image.Image) -> bytes:
    """
    Convert PIL Image to packed binary data.

    Args:
        image: PIL Image (will be converted to 1-bit)

    Returns:
        Packed binary data (4000 bytes)
    """
    # Convert to grayscale then to 1-bit
    if image.mode != 'L':
        image = image.convert('L')

    # Resize if needed
    if image.size != (EPD_WIDTH, EPD_HEIGHT):
        image = image.resize((EPD_WIDTH, EPD_HEIGHT), Image.Resampling.NEAREST)

    # Convert to numpy array
    pixels = np.array(image)

    # Pack to binary
    data = bytearray(EPD_ARRAY)

    for y in range(EPD_HEIGHT):
        for x_byte in range(EPD_WIDTH // 8):
            byte_val = 0
            for bit in range(8):
                x = x_byte * 8 + bit
                # Threshold at 128, MSB = leftmost
                if pixels[y, x] > 127:
                    byte_val |= (1 << (7 - bit))

            byte_idx = y * (EPD_WIDTH // 8) + x_byte
            data[byte_idx] = byte_val

    return bytes(data)


def image_to_bin(image_path: str, output_path: str) -> None:
    """
    Convert PNG image to packed binary file.

    Args:
        image_path: Path to source image (PNG, etc.)
        output_path: Path to save .bin file
    """
    image = Image.open(image_path).convert('L')
    data = pil_to_bin(image)

    with open(output_path, 'wb') as f:
        f.write(data)

    print(f"Saved: {output_path} ({len(data)} bytes)")


def buffer_to_image(buffer: bytearray, output_path: str = None) -> Image.Image:
    """
    Convert in-memory buffer to PIL Image (for testing text_overlay).

    Args:
        buffer: bytearray of packed pixel data (4000 bytes)
        output_path: Optional path to save PNG

    Returns:
        PIL Image object
    """
    image = bin_to_pil(bytes(buffer))

    if output_path:
        image.save(output_path)
        print(f"Saved: {output_path}")

    return image


def main():
    """Command line interface."""
    import argparse

    parser = argparse.ArgumentParser(description="E-ink binary file viewer")
    parser.add_argument('input', help='Input file (.bin or image)')
    parser.add_argument('-o', '--output', help='Output file')
    parser.add_argument('-s', '--show', action='store_true', help='Display the image')

    args = parser.parse_args()

    input_path = Path(args.input)

    if input_path.suffix.lower() == '.bin':
        # Convert bin to image
        output = args.output or input_path.with_suffix('.png')
        bin_to_image(args.input, str(output), show=args.show)
    else:
        # Convert image to bin
        output = args.output or input_path.with_suffix('.bin')
        image_to_bin(args.input, str(output))


if __name__ == "__main__":
    main()
