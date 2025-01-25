# Convert compressed frames to png
import os
import argparse
import cv2
import zstandard as zstd
import numpy as np
from tqdm import tqdm

def convert_bin_files(input_dir, output_dir, width=848, height=480):
    # Create output directories
    depth_dir = os.path.join(output_dir, "depth")
    rgb_dir = os.path.join(output_dir, "rgb")
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(rgb_dir, exist_ok=True)

    # Create Zstandard decompressor
    dctx = zstd.ZstdDecompressor()

    # Process all bin files in input directory
    for bin_file in tqdm(os.listdir(input_dir), desc="Processing files"):
        if not bin_file.endswith(".bin"):
            continue

        file_path = os.path.join(input_dir, bin_file)
        base_name = os.path.splitext(bin_file)[0]

        # Determine file type
        if "depth" in base_name.lower():
            output_subdir = depth_dir
            is_depth = True
        elif "rgb" in base_name.lower():
            output_subdir = rgb_dir
            is_depth = False
        else:
            continue

        # Read and process binary file
        try:
            with open(file_path, "rb") as f:
                frame_count = 0
                
                while True:
                    # Read size header
                    size_bytes = f.read(4)
                    if not size_bytes:
                        break
                    
                    data_size = int.from_bytes(size_bytes, byteorder='little')
                    if data_size == 0:
                        break

                    # Read compressed data
                    compressed_data = f.read(data_size)
                    if len(compressed_data) != data_size:
                        break

                    # Decompress based on file type
                    if is_depth:
                        # Decompress depth frame
                        depth_data = dctx.decompress(compressed_data)
                        depth_frame = np.frombuffer(depth_data, dtype=np.uint16)
                        depth_frame = depth_frame.reshape((height, width))
                        
                        # Normalize for visualization
                        depth_image = cv2.normalize(
                            depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
                        )
                        output_path = os.path.join(
                            output_subdir, 
                            f"{base_name}_frame{frame_count:04d}.png"
                        )
                        cv2.imwrite(output_path, depth_image)
                    else:
                        # Decompress RGB frame
                        rgb_image = cv2.imdecode(
                            np.frombuffer(compressed_data, dtype=np.uint8),
                            cv2.IMREAD_COLOR
                        )
                        output_path = os.path.join(
                            output_subdir,
                            f"{base_name}_frame{frame_count:04d}.png"
                        )
                        cv2.imwrite(output_path, rgb_image)
                    
                    frame_count += 1

        except Exception as e:
            print(f"\nError processing {bin_file}: {str(e)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert depth/RGB bin files to PNG images"
    )
    parser.add_argument(
        "-i", "--input", 
        required=True,
        help="Input directory containing .bin files"
    )
    parser.add_argument(
        "-o", "--output", 
        required=True,
        help="Output directory for PNG images"
    )
    parser.add_argument(
        "--width", 
        type=int, 
        default=848,
        help="Image width (default: 640)"
    )
    parser.add_argument(
        "--height", 
        type=int, 
        default=480,
        help="Image height (default: 480)"
    )

    args = parser.parse_args()

    print(f"Converting files in {args.input} to PNG...")
    convert_bin_files(args.input, args.output, args.width, args.height)
    print("Conversion complete!")