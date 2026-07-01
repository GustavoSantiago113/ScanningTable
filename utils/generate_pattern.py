import random
from PIL import Image, ImageDraw

# ==========================================
# PSEUDO-RANDOM CALIBRATION PATTERN GENERATOR
# ==========================================
# Generates a 130 mm x 130 mm printable calibration pattern.
# Output: PNG image at high resolution.
#
# Features:
# - Deterministic pseudo-random pattern using a seed
# - Adjustable DPI
# - Adjustable grid size
# - Black/white square calibration cells
#
# Requirements:
#   pip install pillow
#
# Usage:
#   python generate_pattern.py
# ==========================================

# ----------------------------
# USER SETTINGS
# ----------------------------
PATTERN_SIZE_MM = 130
DPI = 300
GRID_CELLS = 52          # 52x52 cells
SEED = 42                # Change for a different pattern
BORDER_MM = 5            # White border around pattern
OUTPUT_FILE = "pseudo_random_calibration_pattern.png"
# ----------------------------
# UNIT CONVERSION
# ----------------------------
MM_PER_INCH = 25.4

def generate_pattern(PATTERN_SIZE_MM, DPI, GRID_CELLS, SEED, BORDER_MM):

    pattern_pixels = int((PATTERN_SIZE_MM / MM_PER_INCH) * DPI)
    border_pixels = int((BORDER_MM / MM_PER_INCH) * DPI)
    canvas_size = pattern_pixels + (2 * border_pixels)
    cell_size = pattern_pixels // GRID_CELLS

    # ----------------------------
    # CREATE IMAGE
    # ----------------------------
    img = Image.new("L", (canvas_size, canvas_size), 255)
    # "L" mode = grayscale
    # 255 = white

    # Drawing object
    draw = ImageDraw.Draw(img)

    # ----------------------------
    # RANDOM PATTERN GENERATION
    # ----------------------------
    random.seed(SEED)

    for y in range(GRID_CELLS):
        for x in range(GRID_CELLS):
            # Randomly choose black or white
            value = random.choice([0, 255])

            x0 = border_pixels + x * cell_size
            y0 = border_pixels + y * cell_size
            x1 = x0 + cell_size
            y1 = y0 + cell_size

            draw.rectangle([x0, y0, x1, y1], fill=value)
    
    return img, canvas_size

# ----------------------------
# SAVE IMAGE
# ----------------------------
""" img, canvas_size = generate_pattern(PATTERN_SIZE_MM, DPI, GRID_CELLS, SEED, BORDER_MM)
img.save(OUTPUT_FILE, dpi=(DPI, DPI))

print("====================================")
print("Pseudo-random calibration pattern generated")
print(f"Output file : {OUTPUT_FILE}")
print(f"Pattern size: {PATTERN_SIZE_MM} mm x {PATTERN_SIZE_MM} mm")
print(f"Resolution  : {DPI} DPI")
print(f"Grid        : {GRID_CELLS} x {GRID_CELLS}")
print(f"Canvas size : {canvas_size} x {canvas_size} pixels")
print("====================================") """
