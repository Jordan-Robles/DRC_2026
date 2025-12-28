import cv2
import numpy as np

def nothing(x):
    pass

def create_sliders(color_name, low, high):
    """Create HSV sliders attached to the 'Stream' window."""
    cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Stream', 1000, 500)
    cv2.createTrackbar(f'{color_name}_L-H', 'Stream', low[0], 179, nothing)
    cv2.createTrackbar(f'{color_name}_L-S', 'Stream', low[1], 255, nothing)
    cv2.createTrackbar(f'{color_name}_L-V', 'Stream', low[2], 255, nothing)
    cv2.createTrackbar(f'{color_name}_H-H', 'Stream', high[0], 179, nothing)
    cv2.createTrackbar(f'{color_name}_H-S', 'Stream', high[1], 255, nothing)
    cv2.createTrackbar(f'{color_name}_H-V', 'Stream', high[2], 255, nothing)

def get_slider_values(color_name):
    """Get current HSV slider values for the active color."""
    l_h = cv2.getTrackbarPos(f'{color_name}_L-H', 'Stream')
    l_s = cv2.getTrackbarPos(f'{color_name}_L-S', 'Stream')
    l_v = cv2.getTrackbarPos(f'{color_name}_L-V', 'Stream')
    h_h = cv2.getTrackbarPos(f'{color_name}_H-H', 'Stream')
    h_s = cv2.getTrackbarPos(f'{color_name}_H-S', 'Stream')
    h_v = cv2.getTrackbarPos(f'{color_name}_H-V', 'Stream')
    return np.array([l_h, l_s, l_v]), np.array([h_h, h_s, h_v])

def draw_text(img, text, pos, color=(0, 255, 0), scale=2, thickness=2):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

def main():
    # Use DirectShow backend on Windows for better compatibility
    print("Attempting to open camera...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("Cannot open webcam at index 0 with DirectShow, trying default backend...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open webcam. Please check your camera connection.")
            return
    
    print("Camera opened successfully!")
    print(f"Camera resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    presets = {
        'Yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
        'Blue': (np.array([100, 100, 100]), np.array([130, 255, 255]))
    }

    mode = 1  # Start in combined mode (show values and combined filter)

    # For modes 2 and 3, we need sliders, so create them upfront:
    # We'll create/destroy window on mode change, but start with no sliders for mode 1.
    # We'll create sliders on demand.

    # Track current active color for slider mode
    active_color = None

    print("Controls:")
    print("1: Show combined mask and HSV values (no sliders)")
    print("2: Show Blue sliders and Blue filtered stream")
    print("3: Show Yellow sliders and Yellow filtered stream")
    print("q or ESC: Quit")
    print("\nStarting video stream...")

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Can't receive frame after {frame_count} frames. Exiting ...")
            break
        
        frame_count += 1
        if frame_count == 1:
            print(f"First frame received! Shape: {frame.shape}")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if mode == 1:
            # Combined mask
            lower_yellow, upper_yellow = presets['Yellow']
            lower_blue, upper_blue = presets['Blue']

            # If sliders were ever created, read their values
            if active_color == 'Yellow':
                lower_yellow, upper_yellow = get_slider_values('Yellow')
            elif active_color == 'Blue':
                lower_blue, upper_blue = get_slider_values('Blue')

            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            combined_mask = cv2.bitwise_or(mask_yellow, mask_blue)
            result = cv2.bitwise_and(frame, frame, mask=combined_mask)

            # Display combined stream side by side: Original | Filtered
            combined = np.hstack((frame, result))

            # Create a black canvas for HSV values
            info_canvas = np.zeros((200, combined.shape[1], 3), dtype=np.uint8)

            # Get slider values or preset values for display
            yl_l, yl_h = lower_yellow, upper_yellow
            bl_l, bl_h = lower_blue, upper_blue

            # Draw Yellow HSV values
            draw_text(info_canvas, "Yellow HSV:", (10, 60))
            draw_text(info_canvas, f"Low H: {yl_l[0]} S: {yl_l[1]} V: {yl_l[2]}", (10, 120))
            draw_text(info_canvas, f"High H: {yl_h[0]} S: {yl_h[1]} V: {yl_h[2]}", (10, 180))

            # Draw Blue HSV values
            draw_text(info_canvas, "Blue HSV:", (1000, 60))
            draw_text(info_canvas, f"Low H: {bl_l[0]} S: {bl_l[1]} V: {bl_l[2]}", (1000, 120))
            draw_text(info_canvas, f"High H: {bl_h[0]} S: {bl_h[1]} V: {bl_h[2]}", (1000, 180))

            # Stack combined image and info canvas vertically
            display = np.vstack((info_canvas, combined))

            cv2.imshow('Stream', display)

            # Destroy sliders window if it exists
            if active_color is not None:
                cv2.destroyWindow('Stream')
                cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)
                active_color = None  # Clear active sliders

        elif mode == 2:
            # Blue sliders + Blue mask only
            if active_color != 'Blue':
                cv2.destroyWindow('Stream')
                create_sliders('Blue', *presets['Blue'])
                active_color = 'Blue'

            lower_blue, upper_blue = get_slider_values('Blue')
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            result = cv2.bitwise_and(frame, frame, mask=mask_blue)

            combined = np.hstack((frame, result))
            cv2.imshow('Stream', combined)

        elif mode == 3:
            # Yellow sliders + Yellow mask only
            if active_color != 'Yellow':
                cv2.destroyWindow('Stream')
                create_sliders('Yellow', *presets['Yellow'])
                active_color = 'Yellow'

            lower_yellow, upper_yellow = get_slider_values('Yellow')
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            result = cv2.bitwise_and(frame, frame, mask=mask_yellow)

            combined = np.hstack((frame, result))
            cv2.imshow('Stream', combined)

        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord('q')]:
            break
        elif key == ord('1'):
            mode = 1
        elif key == ord('2'):
            mode = 2
        elif key == ord('3'):
            mode = 3

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
