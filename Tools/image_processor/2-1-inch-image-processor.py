from PIL import Image
import numpy as np
import os

# E-ink display specifications
EPD_WIDTH = 128
EPD_HEIGHT = 250
ROTATION_180 = False  # Set to match your main.py rotation setting

def dump_1bit_eink(pixels: np.ndarray, width: int, height: int, rotation_180: bool = True):
    """
    Convert grayscale image to 1-bit packed format for e-ink display
    - pixels: 2D numpy array of grayscale values
    - width, height: display dimensions  
    - rotation_180: whether display is rotated 180 degrees
    """
    
    # Ensure correct dimensions
    if pixels.shape != (height, width):
        print(f"Warning: Image size {pixels.shape} doesn't match display size ({height}, {width})")
        # Resize if needed
        pil_image = Image.fromarray(pixels)
        pil_image = pil_image.resize((width, height), Image.Resampling.LANCZOS)
        pixels = np.array(pil_image)
        print(f"Resized image to {pixels.shape}")
    
    # Apply horizontal flip to correct left-right mirroring (always applied first)
    pixels = np.fliplr(pixels)  # Flip left-right to correct mirroring
    print("Applied horizontal flip to correct mirroring")
    
    # Apply 180-degree rotation after mirroring (now applied by default)
    if rotation_180:
        pixels = np.rot90(pixels, 2)  # Rotate 180 degrees
        print("Applied 180-degree rotation after horizontal flip")
    
    # Convert to binary: 0 = black pixel, 1 = white pixel
    # E-ink typically uses 0 for black, 1 for white
    binary_pixels = (pixels > 127).astype(np.uint8)
    
    # Pack bits properly for e-ink display
    # E-ink expects data row by row, with each byte containing 8 horizontal pixels
    # MSB (bit 7) = leftmost pixel, LSB (bit 0) = rightmost pixel
    
    packed_data = []
    
    for row in range(height):
        for col_byte in range(width // 8):  # Each byte represents 8 pixels
            byte_val = 0
            for bit in range(8):
                pixel_col = col_byte * 8 + bit
                if pixel_col < width:
                    # MSB first: bit 7 = leftmost pixel
                    if binary_pixels[row, pixel_col]:
                        byte_val |= (1 << (7 - bit))
            packed_data.append(byte_val)
    
    return np.array(packed_data, dtype=np.uint8)

def create_test_pattern(width: int, height: int):
    """Create a test pattern to verify display is working correctly"""
    test_image = np.zeros((height, width), dtype=np.uint8)
    
    # Create border
    test_image[0:5, :] = 255  # Top border
    test_image[-5:, :] = 255  # Bottom border  
    test_image[:, 0:5] = 255  # Left border
    test_image[:, -5:] = 255  # Right border
    
    # Create diagonal lines
    for i in range(min(height, width)):
        if i < height and i < width:
            test_image[i, i] = 255
        if i < height and (width-1-i) >= 0:
            test_image[i, width-1-i] = 255
    
    # Add some text-like rectangles and orientation markers
    test_image[50:70, 20:40] = 255  # Rectangle 1
    test_image[50:70, 50:90] = 255  # Rectangle 2 
    test_image[80:100, 20:60] = 255  # Rectangle 3
    
    # Add "L" shape in top-left to verify orientation (should appear correctly after flip)
    test_image[10:30, 10:15] = 255  # Vertical line of "L"
    test_image[25:30, 10:25] = 255  # Horizontal line of "L"
    
    return test_image

# Process loading1.bin (Image A)
print("=== Processing loading1.bin ===")
image_path_a = "/Users/chengmingzhang/CodingProjects/Software/Distiller-SAM-Firmware/Asset/Loading-A-2-1-inch.png"
output_path_a = '/Users/chengmingzhang/CodingProjects/Software/Distiller-SAM-Firmware/src/V0.2.2/bin/loading1.bin'

try:
    if os.path.exists(image_path_a):
        print(f"Loading image: {image_path_a}")
        image = Image.open(image_path_a).convert("L")
        print(f"Original image size: {image.size}")
        
        # Convert PIL image to numpy array
        image_array = np.array(image)
        print(f"Image array shape: {image_array.shape}")
        
        # Process the image
        converted_pixels = dump_1bit_eink(image_array, EPD_WIDTH, EPD_HEIGHT, ROTATION_180)
        
        # Save loading1 image
        os.makedirs(os.path.dirname(output_path_a), exist_ok=True)
        
        with open(output_path_a, 'wb') as f:
            f.write(converted_pixels.tobytes())
        
        print(f"Loading1 image written to: {output_path_a}")
        print(f"Data size: {len(converted_pixels)} bytes (expected: {EPD_WIDTH * EPD_HEIGHT // 8})")
        
    else:
        print(f"Image file not found: {image_path_a}")
        print("Creating test pattern for loading1.bin...")
        
        # Create test pattern
        test_pattern = create_test_pattern(EPD_WIDTH, EPD_HEIGHT)
        converted_test = dump_1bit_eink(test_pattern, EPD_WIDTH, EPD_HEIGHT, ROTATION_180)
        
        # Save test pattern
        os.makedirs(os.path.dirname(output_path_a), exist_ok=True)
        with open(output_path_a, 'wb') as f:
            f.write(converted_test.tobytes())
        
        print(f"Test pattern written to: {output_path_a}")
        print(f"Data size: {len(converted_test)} bytes")
        
except Exception as e:
    print(f"Error processing loading1.bin: {e}")

# Process loading2.bin (Image B)
print("\n=== Processing loading2.bin ===")
image_path_b = "/Users/chengmingzhang/CodingProjects/Software/Distiller-SAM-Firmware/Asset/Loading-A-2-1-inch.png"
output_path_b = '/Users/chengmingzhang/CodingProjects/Software/Distiller-SAM-Firmware/src/V0.2.2/bin/loading1-180.bin'

try:
    if os.path.exists(image_path_b):
        print(f"Loading image: {image_path_b}")
        image = Image.open(image_path_b).convert("L")
        print(f"Original image size: {image.size}")
        
        # Convert PIL image to numpy array
        image_array = np.array(image)
        print(f"Image array shape: {image_array.shape}")
        
        # Process the image
        converted_pixels = dump_1bit_eink(image_array, EPD_WIDTH, EPD_HEIGHT, ROTATION_180)
        
        # Save loading2 image
        os.makedirs(os.path.dirname(output_path_b), exist_ok=True)
        
        with open(output_path_b, 'wb') as f:
            f.write(converted_pixels.tobytes())
        
        print(f"Loading2 image written to: {output_path_b}")
        print(f"Data size: {len(converted_pixels)} bytes (expected: {EPD_WIDTH * EPD_HEIGHT // 8})")
        
    else:
        print(f"Image file not found: {image_path_b}")
        print("Creating inverted test pattern for loading2.bin...")
        
        # Create inverted test pattern
        test_pattern = create_test_pattern(EPD_WIDTH, EPD_HEIGHT)
        inverted_pattern = 255 - test_pattern  # Invert the pattern
        converted_test = dump_1bit_eink(inverted_pattern, EPD_WIDTH, EPD_HEIGHT, ROTATION_180)
        
        # Save inverted test pattern
        os.makedirs(os.path.dirname(output_path_b), exist_ok=True)
        with open(output_path_b, 'wb') as f:
            f.write(converted_test.tobytes())
        
        print(f"Inverted test pattern written to: {output_path_b}")
        print(f"Data size: {len(converted_test)} bytes")
        
except Exception as e:
    print(f"Error processing loading2.bin: {e}")

print("\n=== Conversion Complete! ===")
print(f"Display settings: {EPD_WIDTH}x{EPD_HEIGHT}, 180° rotation: {ROTATION_180}")
print("Processing order: 1) Horizontal flip (mirroring fix), 2) 180° rotation")
print("Both images processed with horizontal flip + 180° rotation by default.")
print("Make sure these settings match your main.py configuration.")