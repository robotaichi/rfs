#!/usr/bin/env python3
"""
Logic layer for the GUI that displays the FACES IV plot image.

Handles displaying the plot image and toggling the blink effect using Tkinter.
Does not depend on ROS2 communication. Called from the rfs_viewer node.
"""

import os
import tkinter as tk
from PIL import Image, ImageTk


class PlotViewerGUI:
    """Logic class that toggles between the plot image and its background version every second to create a blink effect."""

    def __init__(self, master, initial_plot_path):
        self.master = master
        self.master.title("RFS FACES IV Plot Viewer")
        self.label = tk.Label(self.master)
        self.label.pack(fill=tk.BOTH, expand=True)
        self.plot_path = initial_plot_path
        self.bg_path = initial_plot_path.replace(".png", "_bg.png")
        self.show_bg = False
        self.master.update_idletasks()
        self.update_image()
        self.blink_active = True
        self.master.after(1000, self.toggle_blink)

    def toggle_blink(self):
        """Switch between the normal and background images every second, making the marker appear to blink."""
        if not self.blink_active: return
        self.show_bg = not self.show_bg
        self.update_image()
        self.master.after(1000, self.toggle_blink)

    def update_image(self, path=None):
        """Load and display the image at the given path. If path is "RESET", show a "Resetting..." placeholder instead."""
        if path == "RESET":
            self.label.config(image='', text="Resetting...", font=("Helvetica", 24, "bold"))
            self.label.image = None
            return

        if path:
            self.plot_path = path
            self.bg_path = path.replace(".png", "_bg.png")
            self.label.config(text="")  # Clear placeholder text

        current_path = self.bg_path if self.show_bg else self.plot_path
        if os.path.exists(current_path):
            try:
                img = Image.open(current_path)
                w = self.master.winfo_width()
                h = self.master.winfo_height()
                if w < 100: w = 480  # Default if winfo not ready
                if h < 100: h = 480
                img.thumbnail((w-20, h-20), Image.LANCZOS)
                self.photo = ImageTk.PhotoImage(img)
                self.label.config(image=self.photo)
                self.label.image = self.photo
            except Exception: pass
        else:
            self.label.config(image='', text="Resetting...", font=("Helvetica", 24, "bold"))
            self.label.image = None
