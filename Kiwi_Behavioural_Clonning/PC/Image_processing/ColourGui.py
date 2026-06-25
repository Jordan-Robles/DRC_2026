import os
import sys
import cv2
import json
import numpy as np
from PyQt5.QtCore import QTimer, Qt, QRect, QPoint, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QFormLayout,
    QTabWidget, QSizePolicy, QHBoxLayout, QPushButton, QFileDialog, 
    QCheckBox, QComboBox
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor


class RangeSlider(QWidget):
    lowValueChanged = pyqtSignal(int)
    highValueChanged = pyqtSignal(int)

    def __init__(self, min_val=0, max_val=100, parent=None):
        super().__init__(parent)
        self._min = min_val
        self._max = max_val
        self._low = min_val
        self._high = max_val

        # Initialize dragging flags
        self._dragging_low = False
        self._dragging_high = False

        # Add these defaults to enable proper drawing and event detection:
        self._handle_radius = 8
        self._bar_height = 6

        # Optionally set a default fixed height for the widget so it shows properly:
        self.setFixedHeight(30)

    def setLowValue(self, val):
        if val != self._low and self._min <= val <= self._high:
            self._low = val
            self.lowValueChanged.emit(val)
            self.update()

    def setHighValue(self, val):
        if val != self._high and self._low <= val <= self._max:
            self._high = val
            self.highValueChanged.emit(val)
            self.update()

    def lowValue(self):
        return self._low

    def highValue(self):
        return self._high

    def paintEvent(self, event):
        painter = QPainter(self)
        w = self.width()
        h = self.height()

        # Draw background bar
        bar_rect = QRect(self._handle_radius, h // 2 - self._bar_height // 2,
                         w - 2 * self._handle_radius, self._bar_height)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(200, 200, 200))
        painter.drawRect(bar_rect)

        # Draw selection bar
        total_range = self._max - self._min
        left_pos = int((self._low - self._min) / total_range * (w - 2 * self._handle_radius)) + self._handle_radius
        right_pos = int((self._high - self._min) / total_range * (w - 2 * self._handle_radius)) + self._handle_radius
        selection_rect = QRect(left_pos, h // 2 - self._bar_height // 2, right_pos - left_pos, self._bar_height)
        painter.setBrush(QColor(100, 150, 220))
        painter.drawRect(selection_rect)

        # Draw handles
        painter.setBrush(QColor(50, 100, 180))
        painter.drawEllipse(QPoint(left_pos, h // 2), self._handle_radius, self._handle_radius)
        painter.drawEllipse(QPoint(right_pos, h // 2), self._handle_radius, self._handle_radius)

    def mousePressEvent(self, event):
        pos = event.pos()
        w = self.width()
        total_range = self._max - self._min
        left_pos = int((self._low - self._min) / total_range * (w - 2 * self._handle_radius)) + self._handle_radius
        right_pos = int((self._high - self._min) / total_range * (w - 2 * self._handle_radius)) + self._handle_radius
        if abs(pos.x() - left_pos) <= self._handle_radius:
            self._dragging_low = True
        elif abs(pos.x() - right_pos) <= self._handle_radius:
            self._dragging_high = True

    def mouseMoveEvent(self, event):
        if not (self._dragging_low or self._dragging_high):
            return
        x = event.pos().x()
        w = self.width()
        total_range = self._max - self._min
        x = max(self._handle_radius, min(x, w - self._handle_radius))
        val = int(((x - self._handle_radius) / (w - 2 * self._handle_radius)) * total_range + self._min)
        if self._dragging_low:
            self.setLowValue(val)
        elif self._dragging_high:
            self.setHighValue(val)
        self.update()

    def mouseReleaseEvent(self, event):
        self._dragging_low = False
        self._dragging_high = False


class HSVSlider(QWidget):
    def __init__(self, color_name, low, high):
        super().__init__()
        layout = QFormLayout()
        self.sliders = {}
        self.low_labels = {}
        self.high_labels = {}

        channels = [
            ('H', 0, 179, low[0], high[0]),
            ('S', 0, 255, low[1], high[1]),
            ('V', 0, 255, low[2], high[2])
        ]

        for ch_name, ch_min, ch_max, init_low, init_high in channels:
            container = QWidget()
            h_layout = QHBoxLayout()
            h_layout.setContentsMargins(0, 0, 0, 0)

            label = QLabel(f"{color_name} {ch_name}")
            label.setFixedWidth(60)

            slider = RangeSlider(ch_min, ch_max)
            slider.setLowValue(init_low)
            slider.setHighValue(init_high)
            slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            low_val_label = QLabel(str(init_low))
            low_val_label.setFixedWidth(30)
            low_val_label.setAlignment(Qt.AlignCenter)

            high_val_label = QLabel(str(init_high))
            high_val_label.setFixedWidth(30)
            high_val_label.setAlignment(Qt.AlignCenter)

            # Update labels when slider changes
            slider.lowValueChanged.connect(lambda val, lbl=low_val_label: lbl.setText(str(val)))
            slider.highValueChanged.connect(lambda val, lbl=high_val_label: lbl.setText(str(val)))

            h_layout.addWidget(label)
            h_layout.addWidget(slider)
            h_layout.addWidget(low_val_label)
            h_layout.addWidget(high_val_label)
            container.setLayout(h_layout)

            layout.addRow(container)

            self.sliders[ch_name] = slider
            self.low_labels[ch_name] = low_val_label
            self.high_labels[ch_name] = high_val_label

        self.setLayout(layout)

    def get_values(self):
        low = np.array([
            self.sliders['H'].lowValue(),
            self.sliders['S'].lowValue(),
            self.sliders['V'].lowValue()
        ])
        high = np.array([
            self.sliders['H'].highValue(),
            self.sliders['S'].highValue(),
            self.sliders['V'].highValue()
        ])
        return low, high


class VideoDisplay(QLabel):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setScaledContents(False)
        self._pixmap = None

    def set_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self._pixmap = QPixmap.fromImage(qt_image)
        self._update_scaled_pixmap()

    def resizeEvent(self, event):
        self._update_scaled_pixmap()
        super().resizeEvent(event)

    def _update_scaled_pixmap(self):
        if self._pixmap:
            scaled = self._pixmap.scaled(
                self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.setPixmap(scaled)


class VideoTab(QWidget):
    def __init__(self, color_name=None, presets=None, is_combined=False, main_window=None):
        super().__init__()
        self.color_name = color_name
        self.presets = presets
        self.is_combined = is_combined  # Flag for combined tab
        self.main_window = main_window

        self.video_display = VideoDisplay()
        layout = QVBoxLayout()
        layout.addWidget(self.video_display)

        if self.color_name:
            self.sliders = HSVSlider(color_name, presets[color_name][0], presets[color_name][1])
            layout.addWidget(self.sliders)
        else:
            self.sliders = None

        if self.color_name or self.is_combined:
            self.reset_button = QPushButton("Reset")
            layout.addWidget(self.reset_button)
            self.reset_button.clicked.connect(self.reset_sliders)
        else:
            self.reset_button = None

        if self.is_combined:
            self.load_button = QPushButton("Load Parameters")
            layout.addWidget(self.load_button)

            self.save_button = QPushButton("Save Parameters")
            layout.addWidget(self.save_button)

        else:
            self.save_button = None
            self.load_button = None


        self.setLayout(layout)

    def reset_sliders(self):
        # Reset sliders to last loaded parameters, or presets if nothing loaded
        if self.is_combined:
            # Reset all color tabs sliders
            for color_name, tab in [('Blue', self.main_window.tab_blue),
                                    ('Yellow', self.main_window.tab_yellow),
                                    ('Green', self.main_window.tab_green)]:
                if tab.sliders:
                    if self.main_window.loaded_parameters and color_name in self.main_window.loaded_parameters:
                        # Use loaded parameters
                        low = self.main_window.loaded_parameters[color_name]['low']
                        high = self.main_window.loaded_parameters[color_name]['high']
                    else:
                        # Fall back to presets (tuples)
                        low, high = self.presets[color_name]
                    
                    sliders = tab.sliders.sliders
                    for ch_name, slider in sliders.items():
                        slider.setLowValue(low['HSV'.index(ch_name)])
                        slider.setHighValue(high['HSV'.index(ch_name)])

                    # Update labels immediately
                    for ch_name in ['H', 'S', 'V']:
                        tab.sliders.low_labels[ch_name].setText(str(low['HSV'.index(ch_name)]))
                        tab.sliders.high_labels[ch_name].setText(str(high['HSV'.index(ch_name)]))
        elif self.color_name and self.sliders:
            if self.main_window.loaded_parameters and self.color_name in self.main_window.loaded_parameters:
                # Use loaded parameters
                low = self.main_window.loaded_parameters[self.color_name]['low']
                high = self.main_window.loaded_parameters[self.color_name]['high']
            else:
                # Fall back to presets (tuples)
                low, high = self.presets[self.color_name]
            
            sliders = self.sliders.sliders
            for ch_name, slider in sliders.items():
                slider.setLowValue(low['HSV'.index(ch_name)])
                slider.setHighValue(high['HSV'.index(ch_name)])

            # Update labels immediately
            for ch_name in ['H', 'S', 'V']:
                self.sliders.low_labels[ch_name].setText(str(low['HSV'.index(ch_name)]))
                self.sliders.high_labels[ch_name].setText(str(high['HSV'.index(ch_name)]))
        
        # Check if current values match loaded parameters after reset
        if self.main_window.loaded_parameters:
            self.main_window.check_if_values_match_loaded()



    def get_mask(self, frame):
        if self.sliders is None:
            return None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low, high = self.sliders.get_values()
        mask = cv2.inRange(hsv, low, high)
        return mask

    def update_frame(self, frame):
        if frame is None:
            return
        if self.sliders:
            mask = self.get_mask(frame)
            white_bg = np.full_like(frame, 255)
            filtered = np.where(mask[:, :, None] == 255, frame, white_bg)

        else:
            filtered = frame
        combined = np.hstack((frame, filtered))
        self.video_display.set_frame(combined)

    def update_combined_frame(self, frame, masks):
        # masks is dict of masks from color tabs
        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for mask in masks.values():
            if mask is not None:
                combined_mask = cv2.bitwise_or(combined_mask, mask)

        white_bg = np.full_like(frame, 255)
        filtered = np.where(combined_mask[:, :, None] == 255, frame, white_bg)

        combined = np.hstack((frame, filtered))
        self.video_display.set_frame(combined)



class FilteredOnlyTab(QWidget):
    def __init__(self, presets):
        super().__init__()
        self.presets = presets
        self.video_display = VideoDisplay()

        # Add checkboxes for each color
        self.checkbox_blue = QCheckBox("Show Blue")
        self.checkbox_blue.setChecked(True)
        self.checkbox_yellow = QCheckBox("Show Yellow")
        self.checkbox_yellow.setChecked(True)
        self.checkbox_green = QCheckBox("Show Green")
        self.checkbox_green.setChecked(True)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.video_display)

        # Checkbox layout (horizontal)
        checkbox_layout = QHBoxLayout()
        checkbox_layout.addWidget(self.checkbox_blue)
        checkbox_layout.addWidget(self.checkbox_yellow)
        checkbox_layout.addWidget(self.checkbox_green)
        layout.addLayout(checkbox_layout)

        self.setLayout(layout)

        # Connect checkboxes to update display when toggled
        self.checkbox_blue.toggled.connect(self._on_checkbox_toggled)
        self.checkbox_yellow.toggled.connect(self._on_checkbox_toggled)
        self.checkbox_green.toggled.connect(self._on_checkbox_toggled)

        # Store last frame and masks for quick re-filtering on toggle
        self._last_frame = None
        self._last_masks = {}

    def _on_checkbox_toggled(self):
        # When checkbox toggled, re-filter using last frame and masks if available
        if self._last_frame is not None:
            self.update_filtered_frame(self._last_frame, self._last_masks)

    def update_filtered_frame(self, frame, masks):
        if frame is None:
            return

        # Save last frame and masks
        self._last_frame = frame.copy()
        self._last_masks = masks.copy()

        filtered = np.zeros_like(frame)

        # Add masks only if corresponding checkbox is checked
        if self.checkbox_blue.isChecked() and 'Blue' in masks:
            filtered = cv2.bitwise_or(filtered, cv2.bitwise_and(frame, frame, mask=masks['Blue']))

        if self.checkbox_yellow.isChecked() and 'Yellow' in masks:
            filtered = cv2.bitwise_or(filtered, cv2.bitwise_and(frame, frame, mask=masks['Yellow']))

        if self.checkbox_green.isChecked() and 'Green' in masks:
            filtered = cv2.bitwise_or(filtered, cv2.bitwise_and(frame, frame, mask=masks['Green']))

        # Replace black pixels (background) with white
        black_pixels = (filtered == 0).all(axis=2)
        filtered[black_pixels] = [255, 255, 255]

        self.video_display.set_frame(filtered)




class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        # Find available cameras and create dropdown
        available_cameras = self.get_available_cameras()
        if not available_cameras:
            raise RuntimeError("No cameras found")

        # Create camera selection dropdown
        self.camera_combo = QComboBox()
        for cam_index in available_cameras:
            self.camera_combo.addItem(f"Camera {cam_index}", cam_index)

        # Connect dropdown to camera change handler
        self.camera_combo.currentIndexChanged.connect(self.on_camera_changed)

        # Initialize with first available camera
        self.cap = cv2.VideoCapture(available_cameras[0])
        if not self.cap.isOpened():
            raise RuntimeError("Could not open webcam")

        self.setWindowTitle("Tri-Hard HSV Colour Tuner")
        self.resize(1200, 800)  # Set initial window size (width, height)

        self.presets = {
            'Yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            'Blue': (np.array([100, 100, 100]), np.array([130, 255, 255])),
            'Green': (np.array([40, 70, 70]), np.array([80, 255, 255]))
        }

        self.loaded_parameters = None  # Store last loaded parameters

        self.tabs = QTabWidget()

        self.tab_all = VideoTab(color_name=None, presets=self.presets, is_combined=True, main_window=self)
        self.tabs.addTab(self.tab_all, "All")

        self.tab_filtered_only = FilteredOnlyTab(self.presets)
        self.tabs.addTab(self.tab_filtered_only, "Filtered Only")

        self.tab_blue = VideoTab(color_name='Blue', presets=self.presets, main_window=self)
        self.tabs.addTab(self.tab_blue, "Blue")

        self.tab_yellow = VideoTab(color_name='Yellow', presets=self.presets, main_window=self)
        self.tabs.addTab(self.tab_yellow, "Yellow")

        self.tab_green = VideoTab(color_name='Green', presets=self.presets, main_window=self)
        self.tabs.addTab(self.tab_green, "Green")

        # Create a container for camera selection
        camera_container = QWidget()
        camera_layout = QHBoxLayout()
        camera_layout.setContentsMargins(0, 0, 0, 0)

        camera_label = QLabel("Camera:")
        camera_layout.addWidget(camera_label)
        camera_layout.addWidget(self.camera_combo)
        camera_layout.addStretch()  # Push everything to the left

        camera_container.setLayout(camera_layout)

        layout = QVBoxLayout()
        layout.addWidget(camera_container)  # Add camera selector at top
        layout.addWidget(self.tabs)

        # Add status label below tabs
        self.status_label = QLabel("No parameters loaded")
        layout.addWidget(self.status_label)

        self.setLayout(layout)

        # Track loaded filename and modification status
        self.loaded_filename = None
        self.modified = False

        # Connect save/load buttons on combined tab
        if self.tab_all.save_button:
            self.tab_all.save_button.clicked.connect(self.save_parameters)
        if self.tab_all.load_button:
            self.tab_all.load_button.clicked.connect(self.load_parameters)

        # Connect slider changes to mark modified
        def mark_modified():
            if not self.modified:
                self.modified = True
                self.update_status_label()

        for color_tab in [self.tab_blue, self.tab_yellow]:
            if color_tab.sliders:
                for slider in color_tab.sliders.sliders.values():
                    slider.lowValueChanged.connect(lambda val: mark_modified())
                    slider.highValueChanged.connect(lambda val: mark_modified())

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)
        self.timer.start(30)

    def update_status_label(self):
        if not self.loaded_filename:
            self.status_label.setText("No parameters loaded")
        else:
            status = "modified" if self.modified else "saved"
            display_name = self.loaded_filename.split('/')[-1]
            self.status_label.setText(f"Loaded: {display_name} ({status})")

    def check_if_values_match_loaded(self):
        """Check if current slider values match the loaded parameters"""
        if not self.loaded_parameters:
            return
        
        # Check all color tabs
        for color_name, tab in [('Blue', self.tab_blue), ('Yellow', self.tab_yellow), ('Green', self.tab_green)]:
            if tab.sliders and color_name in self.loaded_parameters:
                current_low, current_high = tab.sliders.get_values()
                loaded_low = self.loaded_parameters[color_name]['low']
                loaded_high = self.loaded_parameters[color_name]['high']
                
                # Compare values
                if (not np.array_equal(current_low, loaded_low) or 
                    not np.array_equal(current_high, loaded_high)):
                    # Values don't match, still modified
                    return
        
        # All values match loaded parameters
        self.modified = False
        self.update_status_label()

    def save_parameters(self):
        params = {}
        for color_name, tab in [('Blue', self.tab_blue), ('Yellow', self.tab_yellow)]:
            if tab.sliders:
                low, high = tab.sliders.get_values()
                params[color_name] = {
                    'low': low.tolist(),
                    'high': high.tolist()
                }

        options = QFileDialog.Options()
        default_dir = self.get_default_save_directory()
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save HSV Parameters",
            default_dir,
            "JSON Files (*.json);;All Files (*)",
            options=options
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(params, f, indent=4)
                print(f"Parameters saved to {filename}")

                # Update status
                self.loaded_filename = filename
                self.modified = False
                self.update_status_label()

            except Exception as e:
                print(f"Error saving parameters: {e}")

    def load_parameters(self):
        options = QFileDialog.Options()
        default_dir = self.get_default_save_directory()
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load HSV Parameters",
            default_dir,
            "JSON Files (*.json);;All Files (*)",
            options=options
        )
        if not filename:
            return

        try:
            with open(filename, 'r') as f:
                params = json.load(f)
            self.loaded_parameters = params.copy()  # Store loaded parameters for reset

            # Update Blue and Yellow sliders if present
            for color_name, tab in [('Blue', self.tab_blue), ('Yellow', self.tab_yellow), ('Green', self.tab_green)]:
                if color_name in params and tab.sliders:
                    low = params[color_name]['low']
                    high = params[color_name]['high']
                    sliders = tab.sliders.sliders
                    for ch_name, slider in sliders.items():
                        slider.setLowValue(low['HSV'.index(ch_name)])
                        slider.setHighValue(high['HSV'.index(ch_name)])

                    # Update labels immediately
                    for ch_name in ['H', 'S', 'V']:
                        tab.sliders.low_labels[ch_name].setText(str(low['HSV'.index(ch_name)]))
                        tab.sliders.high_labels[ch_name].setText(str(high['HSV'.index(ch_name)]))

            print(f"Parameters loaded from {filename}")

            # Update status
            self.loaded_filename = filename
            self.modified = False
            self.update_status_label()

        except Exception as e:
            print(f"Error loading parameters: {e}")


    def update_frames(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        current_tab_text = self.tabs.tabText(self.tabs.currentIndex())

        if current_tab_text == "All":
            # Create masks for all colors including Red2 for full red range
            masks = {}
            for color_name, tab in [('Blue', self.tab_blue),
                                    ('Yellow', self.tab_yellow),
                                    ('Green', self.tab_green)]:
                if tab.sliders is None:
                    continue
                else:
                    mask = tab.get_mask(frame)
                    if mask is not None:
                        masks[color_name] = mask
            self.tab_all.update_combined_frame(frame, masks)
            self.tab_filtered_only.update_filtered_frame(frame, masks)
        elif current_tab_text == "Blue":
            self.tab_blue.update_frame(frame)
        elif current_tab_text == "Yellow":
            self.tab_yellow.update_frame(frame)
        elif current_tab_text == "Filtered Only":
            # Update filtered-only tab with all masks too
            masks = {}
            for color_name, tab in [('Blue', self.tab_blue),
                                    ('Yellow', self.tab_yellow),
                                    ('Green', self.tab_green)]:
                if tab.sliders is None:
                    continue
                else:
                    mask = tab.get_mask(frame)
                    if mask is not None:
                        masks[color_name] = mask
            self.tab_filtered_only.update_filtered_frame(frame, masks)
        elif current_tab_text == "Green":
            self.tab_green.update_frame(frame)
        else:
            self.tab_all.update_frame(frame)

    def get_default_save_directory(self):
        """Get or create the default save directory"""
        # Create parameters directory in the same folder as the script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        params_dir = os.path.join(script_dir, "parameters")
        
        # Create the directory if it doesn't exist
        if not os.path.exists(params_dir):
            os.makedirs(params_dir)
        
        return params_dir
    
    def get_available_cameras(self):
        """Get list of available cameras"""
        available_cameras = []
        for i in range(10):  # Check first 10 camera indices
            test_cap = cv2.VideoCapture(i)
            if test_cap.isOpened():
                available_cameras.append(i)
                test_cap.release()
        return available_cameras
    
    def on_camera_changed(self, index):
        """Handle camera selection change"""
        if self.cap:
            self.cap.release()
        
        camera_index = self.camera_combo.itemData(index)
        self.cap = cv2.VideoCapture(camera_index)
        
        if not self.cap.isOpened():
            print(f"Could not open camera {camera_index}")
            # Try to fall back to camera 0
            self.cap = cv2.VideoCapture(0)

    def closeEvent(self, event):
        self.cap.release()
        super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())