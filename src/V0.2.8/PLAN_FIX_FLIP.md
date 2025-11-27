# Plan: Fix Horizontal Flip for Text Overlay

## Problem

The image-to-binary pipeline in `generate_boot_images.py` applies a horizontal flip (`np.fliplr`) before packing to binary. This means:

1. **Binary data is stored horizontally flipped** relative to the source image
2. The e-ink driver expects this flipped format
3. When `text_overlay.py` draws text onto the buffer, it draws in normal orientation
4. Result: Text appears mirrored on the actual display

## Current Pipeline

```
Source PNG → [flip horizontally] → Pack to binary → Store as .bin
                                                          ↓
                                              e-ink driver reads .bin
                                                          ↓
                                              Display shows correct image
```

## The Issue

```
.bin file (flipped data)
        ↓
text_overlay draws text (normal orientation)
        ↓
Result: Text is backwards relative to the flipped image data
        ↓
e-ink display shows: Image correct, but text is mirrored!
```

## Solution

Two options:

### Option A: Flip text coordinates when drawing (Recommended)
- When drawing at x, calculate flipped x: `flipped_x = (width - 1) - x`
- Also need to flip each character's pixels horizontally
- Text is drawn "backwards" in the buffer, but displays correctly

### Option B: Add flip operation to bin_viewer for preview
- Keep text_overlay drawing normally
- bin_viewer applies reverse flip when converting .bin → PNG
- This way the preview matches what the display shows
- text_overlay still needs to draw flipped for actual device

**Decision: Option A + Option B**
- `text_overlay.py`: Draw text flipped (so it displays correctly on device)
- `bin_viewer.py`: Apply reverse flip when viewing (so preview matches display)

## Implementation

### Step 1: Update `bin_viewer.py`
Add horizontal flip when converting binary to image:
```python
def bin_to_pil(data):
    # ... unpack pixels ...
    # Apply horizontal flip to match what display shows
    pixels = np.fliplr(pixels)
    return Image.fromarray(pixels, mode='L')
```

### Step 2: Update `text_overlay.py`
Flip x-coordinates and character pixels:
```python
def _set_pixel(self, buffer, x, y, color):
    # Flip x coordinate
    x = (self.width - 1) - x
    # ... rest of function ...

def draw_char(self, buffer, char, x, y, color=0):
    # Draw character from right-to-left (flipped)
    # Each row's bits need to be read in reverse order
    ...
```

### Step 3: Test
- Run test_text_overlay.py
- Verify text appears correct in preview images
- The preview should now match what the actual e-ink display shows

## Coordinate System After Fix

```
Buffer (flipped):          Display (what user sees):
x=0 is RIGHT edge    →     x=0 is LEFT edge
x=127 is LEFT edge   →     x=127 is RIGHT edge

When we draw at x=5:
- We flip to x=122 in buffer
- Display shows text at x=5 (left side)
```

## Files to Modify

1. `bin_viewer.py` - Add `np.fliplr(pixels)` in `bin_to_pil()`
2. `text_overlay.py` - Flip x coordinate in `_set_pixel()` and adjust character drawing
