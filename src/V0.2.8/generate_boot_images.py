#!/usr/bin/env python3
"""
Generate boot screen images with version overlay for SAM firmware.

This script:
1. Reads version_config.json for version info and overlay settings
2. Loads base images from Asset folder
3. Overlays text (version numbers) using specified font
4. Saves intermediate PNGs to asset/ folder
5. Converts to e-ink binary format and saves to bin/ folder

Can be run standalone or called from upload.py.

Author: PamirAI
"""

import os
import sys
import json
import argparse
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Please install required packages: pip install Pillow numpy")
    sys.exit(1)


# Path constants
SCRIPT_DIR = Path(__file__).resolve().parent
ASSET_DIR = SCRIPT_DIR.parent.parent / "Asset"
CONFIG_FILE = SCRIPT_DIR / "version_config.json"
OUTPUT_ASSET_DIR = SCRIPT_DIR / "asset"
OUTPUT_BIN_DIR = SCRIPT_DIR / "bin"
DEFAULT_FONT_PATH = ASSET_DIR / "5x5_pixel.ttf"


def load_config(config_path: Path = None) -> dict:
    """Load version_config.json"""
    if config_path is None:
        config_path = CONFIG_FILE

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, 'r') as f:
        return json.load(f)


def overlay_text(
    image: Image.Image,
    text: str,
    x: int,
    y: int,
    font_path: Path,
    font_size: int = 16,
    color: str = "black",
    bg_color: str = None
) -> Image.Image:
    """
    Overlay text on image at specified position (pixel-perfect, no antialiasing).
    Text is rotated 90 degrees counter-clockwise by default.

    Args:
        image: PIL Image to draw on
        text: Text string to overlay
        x: X coordinate for text position
        y: Y coordinate for text position
        font_path: Path to font file
        font_size: Font size in pixels
        color: Text color ("black" or "white")
        bg_color: Background color ("black", "white", or None for opposite of text color)

    Returns:
        Image with text overlay
    """
    # Make a copy to avoid modifying original
    img = image.copy().convert('L')

    # Load font
    try:
        font = ImageFont.truetype(str(font_path), font_size)
    except Exception as e:
        print(f"Warning: Could not load font {font_path}: {e}")
        print("Using default font")
        font = ImageFont.load_default()

    # Convert text color string to 1-bit value (0 = black, 1 = white)
    if color.lower() == "black":
        text_1bit = 0
    elif color.lower() == "white":
        text_1bit = 1
    else:
        try:
            text_1bit = 0 if int(color) < 128 else 1
        except ValueError:
            text_1bit = 0

    # Convert background color string to 1-bit value
    if bg_color is None:
        # Default: opposite of text color
        bg_1bit = 1 if text_1bit == 0 else 0
    elif bg_color.lower() == "black":
        bg_1bit = 0
    elif bg_color.lower() == "white":
        bg_1bit = 1
    else:
        try:
            bg_1bit = 0 if int(bg_color) < 128 else 1
        except ValueError:
            bg_1bit = 1 if text_1bit == 0 else 0

    # Create a temporary 1-bit image for the text (no antialiasing possible in 1-bit mode)
    # Use tight bounding box with minimal padding (1 pixel)
    bbox = font.getbbox(text)
    padding = 1
    text_width = bbox[2] - bbox[0] + padding * 2
    text_height = bbox[3] - bbox[1] + padding * 2

    txt_img = Image.new('1', (text_width, text_height), bg_1bit)
    txt_draw = ImageDraw.Draw(txt_img)
    txt_draw.text((padding - bbox[0], padding - bbox[1]), text, font=font, fill=text_1bit)

    # Rotate text 90 degrees counter-clockwise (left)
    txt_img_rotated = txt_img.rotate(90, expand=True)

    # Convert rotated 1-bit image to grayscale for pasting
    txt_img_gray = txt_img_rotated.convert('L')

    # Paste directly onto the image at position (x, y)
    img.paste(txt_img_gray, (x, y))

    return img


def dump_1bit_eink(pixels: np.ndarray, width: int, height: int, rotation_180: bool = False) -> np.ndarray:
    """
    Convert grayscale image to 1-bit packed format for e-ink display.

    Args:
        pixels: 2D numpy array of grayscale values
        width: Display width
        height: Display height
        rotation_180: Whether to apply 180 degree rotation

    Returns:
        Packed binary data as numpy array
    """
    # Ensure correct dimensions
    if pixels.shape != (height, width):
        print(f"Warning: Image size {pixels.shape} doesn't match display size ({height}, {width})")
        pil_image = Image.fromarray(pixels)
        pil_image = pil_image.resize((width, height), Image.Resampling.LANCZOS)
        pixels = np.array(pil_image)
        print(f"Resized image to {pixels.shape}")

    # Apply horizontal flip to correct left-right mirroring (always applied first)
    pixels = np.fliplr(pixels)

    # Apply 180-degree rotation after mirroring if requested
    if rotation_180:
        pixels = np.rot90(pixels, 2)

    # Convert to binary: 0 = black pixel, 1 = white pixel
    binary_pixels = (pixels > 127).astype(np.uint8)

    # Pack bits for e-ink display
    # Each byte contains 8 horizontal pixels, MSB = leftmost pixel
    packed_data = []

    for row in range(height):
        for col_byte in range(width // 8):
            byte_val = 0
            for bit in range(8):
                pixel_col = col_byte * 8 + bit
                if pixel_col < width:
                    if binary_pixels[row, pixel_col]:
                        byte_val |= (1 << (7 - bit))
            packed_data.append(byte_val)

    return np.array(packed_data, dtype=np.uint8)


def convert_to_eink_bin(image: Image.Image, output_path: Path, width: int, height: int, rotation_180: bool = False):
    """
    Convert PIL image to e-ink binary format and save.

    Args:
        image: PIL Image to convert
        output_path: Path to save binary file
        width: Display width
        height: Display height
        rotation_180: Whether to apply 180 degree rotation
    """
    # Convert to grayscale if needed
    if image.mode != 'L':
        image = image.convert('L')

    # Convert to numpy array
    image_array = np.array(image)

    # Convert to e-ink format
    packed_data = dump_1bit_eink(image_array, width, height, rotation_180)

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Write binary file
    with open(output_path, 'wb') as f:
        f.write(packed_data.tobytes())

    expected_size = width * height // 8
    print(f"  Binary saved: {output_path} ({len(packed_data)} bytes, expected: {expected_size})")


def format_text(text: str, config: dict) -> str:
    """
    Format text string with version placeholders.

    Args:
        text: Text with placeholders like {sam_version}
        config: Config dict containing version values

    Returns:
        Formatted text string
    """
    return text.format(
        sam_version=config.get('sam_version', ''),
        hw_version=config.get('hw_version', '')
    )


def generate_boot_images(
    config_path: Path = None,
    asset_output_dir: Path = None,
    bin_output_dir: Path = None,
    font_path: Path = None,
    verbose: bool = True
) -> bool:
    """
    Main function to generate all boot images.

    Args:
        config_path: Path to version_config.json (default: script dir)
        asset_output_dir: Path to save intermediate PNGs (default: script dir/asset)
        bin_output_dir: Path to save binary files (default: script dir/bin)
        font_path: Path to font file (default: Asset/MedodicaRegular.otf)
        verbose: Print progress messages

    Returns:
        True if successful, False otherwise
    """
    # Set defaults
    if config_path is None:
        config_path = CONFIG_FILE
    if asset_output_dir is None:
        asset_output_dir = OUTPUT_ASSET_DIR
    if bin_output_dir is None:
        bin_output_dir = OUTPUT_BIN_DIR

    try:
        # Load config
        if verbose:
            print(f"Loading config from: {config_path}")
        config = load_config(config_path)

        # Get font path from config or use default
        if font_path is None:
            config_font_path = config.get('font_path')
            if config_font_path:
                font_path = Path(config_font_path)
            else:
                font_path = DEFAULT_FONT_PATH

        # Get display settings
        display = config.get('display', {})
        width = display.get('width', 128)
        height = display.get('height', 250)
        rotation_180 = display.get('rotation_180', False)

        # Get base images config
        base_images = config.get('base_images', {})
        if not base_images:
            print("Error: No base_images defined in config")
            return False

        # Get text overlays
        text_overlays = config.get('text_overlays', [])

        if verbose:
            print(f"SAM Version: {config.get('sam_version', 'N/A')}")
            print(f"HW Version: {config.get('hw_version', 'N/A')}")
            print(f"Display: {width}x{height}, rotation_180: {rotation_180}")
            print(f"Text overlays: {len(text_overlays)}")
            print()

        # Create output directories
        asset_output_dir.mkdir(parents=True, exist_ok=True)
        bin_output_dir.mkdir(parents=True, exist_ok=True)

        # Process each image pair
        for output_name, source_filename in base_images.items():
            if verbose:
                print(f"Processing {output_name}...")

            # Load base image
            source_path = ASSET_DIR / source_filename
            if not source_path.exists():
                print(f"Error: Base image not found: {source_path}")
                return False

            image = Image.open(source_path).convert('L')
            if verbose:
                print(f"  Loaded: {source_path} ({image.size})")

            # Apply all text overlays
            for overlay in text_overlays:
                text = format_text(overlay.get('text', ''), config)
                x = overlay.get('x', 0)
                y = overlay.get('y', 0)
                font_size = overlay.get('font_size', 16)
                color = overlay.get('color', 'black')
                bg_color = overlay.get('bg_color', None)

                image = overlay_text(image, text, x, y, font_path, font_size, color, bg_color)
                if verbose:
                    bg_str = bg_color if bg_color else "auto"
                    print(f"  Overlay: '{text}' at ({x}, {y}), size={font_size}, color={color}, bg={bg_str}")

            # Save intermediate PNG
            png_output_path = asset_output_dir / f"{output_name}.png"
            image.save(png_output_path)
            if verbose:
                print(f"  PNG saved: {png_output_path}")

            # Convert to binary and save
            # Output names MUST be loading1.bin and loading2.bin
            bin_output_path = bin_output_dir / f"{output_name}.bin"
            convert_to_eink_bin(image, bin_output_path, width, height, rotation_180)

        if verbose:
            print()
            print("Boot image generation completed successfully!")

        return True

    except Exception as e:
        print(f"Error generating boot images: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main entry point for command line usage."""
    parser = argparse.ArgumentParser(
        description="Generate boot screen images with version overlay for SAM firmware"
    )
    parser.add_argument(
        '--config', '-c',
        type=Path,
        default=None,
        help='Path to version_config.json (default: script directory)'
    )
    parser.add_argument(
        '--asset-dir', '-a',
        type=Path,
        default=None,
        help='Output directory for intermediate PNGs (default: script dir/asset)'
    )
    parser.add_argument(
        '--bin-dir', '-b',
        type=Path,
        default=None,
        help='Output directory for binary files (default: script dir/bin)'
    )
    parser.add_argument(
        '--font', '-f',
        type=Path,
        default=None,
        help='Path to font file (default: Asset/MedodicaRegular.otf)'
    )
    parser.add_argument(
        '--quiet', '-q',
        action='store_true',
        help='Suppress progress messages'
    )

    args = parser.parse_args()

    success = generate_boot_images(
        config_path=args.config,
        asset_output_dir=args.asset_dir,
        bin_output_dir=args.bin_dir,
        font_path=args.font,
        verbose=not args.quiet
    )

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
