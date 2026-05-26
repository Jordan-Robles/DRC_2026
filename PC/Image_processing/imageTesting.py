import cv2
import numpy as np
import pandas as pd
import os
from pathlib import Path
from ColourFilter import img_preprocess

def browse_images(csv_path, image_folder, white_output=False, delay_ms=50):
    """
    Browse through images in a folder with preprocessing applied.
    """
    # Load CSV to get image names and steering angles
    df = pd.read_csv(csv_path)
    image_files = df['image'].values if 'image' in df.columns else [f for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png'))]
    
    index = 0
    paused = False
    window_name = "Image Browser"
    
    while True:
        # Load and preprocess image
        img_path = os.path.join(image_folder, image_files[index])
        original = cv2.imread(img_path)
        
        if original is None:
            print(f"Failed to load {img_path}")
            index = (index + 1) % len(image_files)
            continue
        
        # Apply preprocessing
        processed = img_preprocess(original, blackWhite =True, lanes= True)
        
        original_resized = cv2.resize(original, (200, 66))
        # Stack side by side for comparison
        display = np.hstack([original_resized, (processed * 255).astype(np.uint8)])
        
        # Get steering angle if available
        info = f"Image {index + 1}/{len(image_files)}: {image_files[index]}"
        if 'steering' in df.columns:
            steering = df['steering'].iloc[index]
            info += f" | Steering: {steering:.3f}"
        
        # Add text to image instead of window title
        cv2.putText(display, info, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        scale = 4  # Increase to 5, 6, etc. for even larger
        display = cv2.resize(display, (display.shape[1] * scale, display.shape[0] * scale))
        cv2.imshow(window_name, display)  # Use fixed window name
        
        # Keyboard controls
        key = cv2.waitKey(delay_ms if not paused else 0) & 0xFF
        if key == 27:  # ESC to exit
            break
        elif key == ord(' '):  # Space to pause/resume
            paused = not paused
        elif key == ord('n') or key == 83:  # n or right arrow
            index = (index + 1) % len(image_files)
        elif key == ord('p') or key == 81:  # p or left arrow
            index = (index - 1) % len(image_files)
        elif key == ord('q'):  # q to exit
            break
        else:
            if not paused:
                index = (index + 1) % len(image_files)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Example usage - adjust paths to your data
    # csv_file = r"C:\Users\jorda\Desktop\Code\Python\DRC_2025\DRC_2025\Testing_Data\Test45\labels.csv"
    # img_folder = r"C:\Users\jorda\Desktop\Code\Python\DRC_2025\DRC_2025\Testing_Data\Test45"

    csv_file = r"C:\Users\jorda\DRC_2026\DRC_2026\Testing_Data\Test6\labels.csv" 
    img_folder = r"C:\Users\jorda\DRC_2026\DRC_2026\Testing_Data\Test6"
    
    browse_images(csv_file, img_folder, white_output=True)