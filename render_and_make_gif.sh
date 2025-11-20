#!/bin/bash

# Default Configuration
RAYTRACER="./raytracer"
FRAMERATE=30
MAX_PARALLEL=8  # Number of scenes to render in parallel

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to display usage
usage() {
    echo "Usage: $0 <scene_pattern> <output_name> [start_frame] [end_frame] [framerate]"
    echo ""
    echo "Arguments:"
    echo "  scene_pattern  : Path pattern to scene files without numbers/extension"
    echo "                   (e.g., 'blog/hw2/inputs/raven/camera_around_david/davids_camera_')"
    echo "  output_name    : Name for output folder and GIF (e.g., 'camera_around')"
    echo "  start_frame    : Starting frame number (default: 0)"
    echo "  end_frame      : Ending frame number (default: 359)"
    echo "  framerate      : Animation framerate (default: 30)"
    echo ""
    echo "Examples:"
    echo "  $0 'blog/hw2/inputs/raven/camera_around_david/davids_camera_' camera_around"
    echo "  $0 'blog/hw2/inputs/raven/camera_zoom_david/davids_camera_zoom_' camera_zoom 0 359"
    echo "  $0 'blog/hw2/inputs/raven/light_around_david/davids_' light_around 0 359 60"
    echo ""
    exit 1
}

# Check arguments
if [ $# -lt 2 ]; then
    usage
fi

SCENE_PATTERN="$1"
OUTPUT_NAME="$2"
START_FRAME=${3:-0}
END_FRAME=${4:-359}

# Override framerate if provided
if [ $# -ge 5 ]; then
    FRAMERATE=$5
fi

OUTPUT_DIR="renders_${OUTPUT_NAME}"
OUTPUT_GIF="${OUTPUT_NAME}.gif"

echo "========================================="
echo "Ray Tracer Scene Renderer & GIF Creator"
echo "========================================="
echo "Scene Pattern: ${SCENE_PATTERN}###.json"
echo "Output Name:   ${OUTPUT_NAME}"
echo "Frame Range:   ${START_FRAME} to ${END_FRAME}"
echo "Framerate:     ${FRAMERATE} fps"
echo "Output Dir:    ${OUTPUT_DIR}"
echo "Output GIF:    ${OUTPUT_GIF}"
echo "========================================="
echo ""

# Check if raytracer executable exists
if [ ! -f "$RAYTRACER" ]; then
    echo -e "${RED}Error: Raytracer executable not found at $RAYTRACER${NC}"
    echo "Please make sure your raytracer is compiled and in the current directory"
    exit 1
fi

# Count available scene files
scene_count=0
for i in $(seq $START_FRAME $END_FRAME); do
    scene_file=$(printf "${SCENE_PATTERN}%03d.json" $i)
    if [ -f "$scene_file" ]; then
        ((scene_count++))
    fi
done

if [ $scene_count -eq 0 ]; then
    echo -e "${RED}Error: No scene files found matching pattern: ${SCENE_PATTERN}###.json${NC}"
    echo -e "${RED}Looking in range: $START_FRAME to $END_FRAME${NC}"
    exit 1
fi

echo -e "${GREEN}Found $scene_count scene files${NC}"
echo ""

# Create output directory
if [ ! -d "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    echo -e "${GREEN}Created output directory: $OUTPUT_DIR${NC}"
fi
echo ""

# Step 1: Render all scenes
echo "Step 1: Rendering scenes..."
echo "----------------------------"
rendered=0

for i in $(seq $START_FRAME $END_FRAME); do
    scene_file=$(printf "${SCENE_PATTERN}%03d.json" $i)
    if [ -f "$scene_file" ]; then
        echo "Rendering $scene_file... ($((rendered+1))/$scene_count)"
        $RAYTRACER "$scene_file" &
        
        # Limit concurrent processes
        if (( $(jobs -r | wc -l) >= MAX_PARALLEL )); then
            wait -n
        fi
        
        ((rendered++))
    fi
done

# Wait for all rendering to complete
echo "Waiting for all renders to complete..."
wait

echo -e "${GREEN}✓ All scenes rendered!${NC}"

# Extract the base filename pattern for the rendered PNGs
# The raytracer outputs PNGs based on the camera.image_name in the JSON
# We need to figure out what pattern the PNGs will have
# Typically they follow the same naming as the JSON files but with .png
SCENE_DIR=$(dirname "$SCENE_PATTERN")
SCENE_BASE=$(basename "$SCENE_PATTERN")

# Move all rendered PNG files to output directory
echo "Moving rendered images to $OUTPUT_DIR/..."
# We'll look for PNGs that match the pattern
moved_count=0
for i in $(seq $START_FRAME $END_FRAME); do
    # Try to find the corresponding PNG file
    png_file=$(printf "${SCENE_BASE}%03d.png" $i)
    if [ -f "$png_file" ]; then
        mv "$png_file" "$OUTPUT_DIR/"
        ((moved_count++))
    fi
done

if [ $moved_count -eq 0 ]; then
    echo -e "${YELLOW}Warning: No PNG files found in current directory${NC}"
    echo -e "${YELLOW}Your raytracer might output PNGs with different names or to a different location${NC}"
    echo -e "${YELLOW}Please check where the PNG files are being created${NC}"
else
    echo -e "${GREEN}✓ Moved $moved_count images to $OUTPUT_DIR/${NC}"
fi
echo ""

# Step 2: Create GIF
echo "Step 2: Creating GIF..."
echo "----------------------------"

# Check if we have any PNG files in the output directory
png_count=$(ls -1 "$OUTPUT_DIR"/*.png 2>/dev/null | wc -l)
if [ $png_count -eq 0 ]; then
    echo -e "${RED}Error: No PNG files found in $OUTPUT_DIR/${NC}"
    echo -e "${RED}Cannot create GIF without rendered images${NC}"
    exit 1
fi

echo -e "${GREEN}Found $png_count PNG files in $OUTPUT_DIR/${NC}"

# Check which tool is available
if command -v ffmpeg &> /dev/null; then
    echo "Using ffmpeg to create GIF..."
    
    # Create high-quality GIF using ffmpeg
    # First create a palette for better colors
    ffmpeg -y -framerate $FRAMERATE -pattern_type glob -i "$OUTPUT_DIR/*.png" \
        -vf "fps=$FRAMERATE,scale=720:-1:flags=lanczos,palettegen" \
        palette.png 2>/dev/null
    
    # Then create the GIF using the palette
    ffmpeg -y -framerate $FRAMERATE -pattern_type glob -i "$OUTPUT_DIR/*.png" \
        -i palette.png \
        -lavfi "fps=$FRAMERATE,scale=720:-1:flags=lanczos [x]; [x][1:v] paletteuse" \
        $OUTPUT_GIF 2>/dev/null
    
    # Clean up palette
    rm -f palette.png
    
    echo -e "${GREEN}✓ GIF created: $OUTPUT_GIF${NC}"
    
elif command -v convert &> /dev/null; then
    echo "Using ImageMagick to create GIF..."
    
    # Calculate delay (in 1/100th of a second)
    delay=$((100 / FRAMERATE))
    
    convert -delay $delay -loop 0 "$OUTPUT_DIR"/*.png $OUTPUT_GIF
    
    echo -e "${GREEN}✓ GIF created: $OUTPUT_GIF${NC}"
    
else
    echo -e "${RED}Error: Neither ffmpeg nor ImageMagick (convert) found!${NC}"
    echo "Please install one of them:"
    echo "  - ffmpeg: sudo apt-get install ffmpeg"
    echo "  - ImageMagick: sudo apt-get install imagemagick"
    exit 1
fi

# Get file size
if [ -f "$OUTPUT_GIF" ]; then
    size=$(du -h "$OUTPUT_GIF" | cut -f1)
    echo -e "${GREEN}✓ GIF size: $size${NC}"
fi

echo ""
echo "========================================="
echo "Done! Your animation is ready:"
echo "  $OUTPUT_GIF"
echo "========================================="

# Optional: Ask if user wants to clean up PNG files
echo ""
read -p "Do you want to delete the rendered PNG files in $OUTPUT_DIR/? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    rm -f "$OUTPUT_DIR"/*.png
    echo -e "${GREEN}✓ PNG files cleaned up${NC}"
fi
