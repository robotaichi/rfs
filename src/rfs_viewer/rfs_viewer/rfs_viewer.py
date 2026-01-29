#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from PIL import Image, ImageTk
import os
import threading
import argparse
from ament_index_python.packages import get_package_share_directory

class PlotViewerGUI:
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
        if not self.blink_active: return
        self.show_bg = not self.show_bg
        self.update_image()
        self.master.after(1000, self.toggle_blink)

    def update_image(self, path=None):
        if path == "RESET":
            self.label.config(image='', text="Resetting...", font=("Helvetica", 24, "bold"))
            self.label.image = None
            return

        if path:
            self.plot_path = path
            self.bg_path = path.replace(".png", "_bg.png")
            self.label.config(text="") # Clear placeholder text

        current_path = self.bg_path if self.show_bg else self.plot_path
        if os.path.exists(current_path):
            try:
                img = Image.open(current_path)
                w = self.master.winfo_width()
                h = self.master.winfo_height()
                if w < 100: w = 480 # Default if winfo not ready
                if h < 100: h = 480
                img.thumbnail((w-20, h-20), Image.LANCZOS)
                self.photo = ImageTk.PhotoImage(img)
                self.label.config(image=self.photo)
                self.label.image = self.photo
            except Exception: pass
        else:
            self.label.config(image='', text="Resetting...", font=("Helvetica", 24, "bold"))
            self.label.image = None

class RFSViewer(Node):
    def __init__(self, gui):
        super().__init__('rfs_viewer')
        self.gui = gui
        self.subscription = self.create_subscription(String, 'rfs_faces_plot_updated', self.listener_callback, 10)
        self.get_logger().info('RFS Viewer Node started.')

    def listener_callback(self, msg):
        self.gui.master.after(0, self.gui.update_image, msg.data)

def main():
    rclpy.init()
    HOME = os.path.expanduser("~")
    DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")
    if not os.path.isdir(DB_DIR):
        try:
            DB_DIR = os.path.join(get_package_share_directory('rfs_config'), 'rfs_database')
        except Exception:
            pass
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", type=str, default="")
    args, _ = parser.parse_known_args()
    
    default_plot_path = os.path.join(DB_DIR, "evaluation_plot.png")
    
    root = tk.Tk()
    if args.geometry:
        root.geometry(args.geometry)
    else:
        root.geometry("200x200")
    gui = PlotViewerGUI(root, default_plot_path)
    node = RFSViewer(gui)
    
    # Run rclpy.spin in a separate thread but keep a reference to node
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful shutdown to prevent 'terminate' crash
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
