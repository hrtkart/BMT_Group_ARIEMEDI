#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import math
from time import time
import threading

from ariemedi_tracker.msg import ToolTrackingData

class MarkerTestPlotter(Node):
    def __init__(self):
        super().__init__('marker_test_plotter')
        self.subscription = self.create_subscription(
            ToolTrackingData,
            '/ARMDpos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.start_time = None
        self.duration = 60.0 # 1 minute
        
        # Tool relative to tracker
        self.times = []
        self.tx = []
        self.ty = []
        self.tz = []
        self.dist = []
        
        # Markers relative to first frame
        self.marker_positions = []  # List of lists: [time_idx][marker_idx][x,y,z]
        self.first_frame_markers = None  # First frame marker positions
        self.relative_positions = []  # Relative positions over time
        
        self.done = False
        
    def listener_callback(self, msg):
        if self.done:
            return
            
        current_time = time()
        if self.start_time is None:
            self.start_time = current_time
            self.get_logger().info('Started recording data for 1 minute...')
            
        elapsed = current_time - self.start_time
        
        if elapsed > self.duration:
            self.done = True
            self.get_logger().info('Finished recording. Plotting data...')
            # Process and Plot plot in background so we don't block
            threading.Thread(target=self.plot_data).start()
            return
            
        # Record time
        self.times.append(elapsed)
        
        # Record tool position (relative to tracker module)
        # Convert from mm to cm for better visualization
        tx, ty, tz = msg.transform.tx / 10.0, msg.transform.ty / 10.0, msg.transform.tz / 10.0
        self.tx.append(tx)
        self.ty.append(ty)
        self.tz.append(tz)
        magnitude = math.sqrt(tx**2 + ty**2 + tz**2)
        self.dist.append(magnitude)
        
        # Record individual markers from the tool plane
        # msg.plane.markers contains the markers on the tracked tool
        current_markers = []
        for pt in msg.plane.markers:
            # pt.p is float64[4], usually p[0]=x, p[1]=y, p[2]=z
            current_markers.append((pt.p[0], pt.p[1], pt.p[2]))
        
        # Store first frame markers as reference
        if self.first_frame_markers is None and len(current_markers) > 0:
            self.first_frame_markers = current_markers.copy()
            self.get_logger().info(f'Set reference frame with {len(current_markers)} markers')
        
        # Calculate relative positions if we have reference frame
        if self.first_frame_markers is not None and len(current_markers) == len(self.first_frame_markers):
            frame_relatives = []
            for i, (cx, cy, cz) in enumerate(current_markers):
                if i < len(self.first_frame_markers):
                    fx, fy, fz = self.first_frame_markers[i]
                    # Calculate relative position (current - first)
                    rel_x = (cx - fx) / 10.0  # Convert to cm
                    rel_y = (cy - fy) / 10.0
                    rel_z = (cz - fz) / 10.0
                    frame_relatives.append((rel_x, rel_y, rel_z))
                else:
                    frame_relatives.append((0.0, 0.0, 0.0))
            self.relative_positions.append(frame_relatives)
        
        self.marker_positions.append(current_markers)
        
        self.get_logger().debug(f"Recorded {len(current_markers)} tool markers")

    def plot_data(self):
        if len(self.times) == 0:
            self.get_logger().warn('No data received!')
            rclpy.shutdown()
            return
                    # Print some statistics
        self.get_logger().info(f"Recorded {len(self.times)} data points")
        self.get_logger().info(f"X range: {min(self.tx):.2f} to {max(self.tx):.2f} cm")
        self.get_logger().info(f"Y range: {min(self.ty):.2f} to {max(self.ty):.2f} cm") 
        self.get_logger().info(f"Z range: {min(self.tz):.2f} to {max(self.tz):.2f} cm")
        self.get_logger().info(f"Total distance range: {min(self.dist):.2f} to {max(self.dist):.2f} cm")
                # Figure 1: Marker (Tool) relative to tracker
        plt.figure("Figure 1: Tool Relative to Tracker", figsize=(10, 8))
        
        plt.subplot(4, 1, 1)
        plt.plot(self.times, self.tx, 'r-', label='X Position')
        plt.ylabel('X (cm)')
        plt.grid(True)
        plt.legend(loc='upper right')
        plt.title('Tool Relative to Tracker Over 1 Min')
        
        plt.subplot(4, 1, 2)
        plt.plot(self.times, self.ty, 'g-', label='Y Position')
        plt.ylabel('Y (cm)')
        plt.grid(True)
        plt.legend(loc='upper right')
        
        plt.subplot(4, 1, 3)
        plt.plot(self.times, self.tz, 'b-', label='Z Position')
        plt.ylabel('Z (cm)')
        plt.grid(True)
        plt.legend(loc='upper right')
        
        plt.subplot(4, 1, 4)
        plt.plot(self.times, self.dist, 'k-', label='Total Distance')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (cm)')
        plt.grid(True)
        plt.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig('/tmp/tool_relative_plot.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        # Figure 2: Marker positions relative to first frame
        if len(self.relative_positions) > 0 and len(self.relative_positions[0]) > 0:
            plt.figure("Figure 2: Marker Positions Relative to First Frame", figsize=(12, 10))
            
            num_markers = len(self.relative_positions[0])
            
            for marker_idx in range(num_markers):
                x_vals = []
                y_vals = []
                z_vals = []
                dist_vals = []
                valid_times = []
                
                for t_idx, frame_relatives in enumerate(self.relative_positions):
                    if marker_idx < len(frame_relatives):
                        rel_x, rel_y, rel_z = frame_relatives[marker_idx]
                        x_vals.append(rel_x)
                        y_vals.append(rel_y)
                        z_vals.append(rel_z)
                        # Calculate total distance for this marker
                        dist = math.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
                        dist_vals.append(dist)
                        valid_times.append(self.times[t_idx])
                
                if len(valid_times) > 0:
                    plt.subplot(4, 1, 1)
                    plt.plot(valid_times, x_vals, label=f'Marker {marker_idx} X')
                    plt.ylabel('X (cm)')
                    plt.grid(True)
                    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
                    
                    plt.subplot(4, 1, 2)
                    plt.plot(valid_times, y_vals, label=f'Marker {marker_idx} Y')
                    plt.ylabel('Y (cm)')
                    plt.grid(True)
                    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
                    
                    plt.subplot(4, 1, 3)
                    plt.plot(valid_times, z_vals, label=f'Marker {marker_idx} Z')
                    plt.ylabel('Z (cm)')
                    plt.grid(True)
                    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
                    
                    plt.subplot(4, 1, 4)
                    plt.plot(valid_times, dist_vals, label=f'Marker {marker_idx} Total Distance')
                    plt.xlabel('Time (s)')
                    plt.ylabel('Distance (cm)')
                    plt.grid(True)
                    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
            
            plt.suptitle('Marker Positions Relative to First Frame Over 1 Min')
            plt.tight_layout()
            plt.savefig('/tmp/marker_relative_plot.png', dpi=150, bbox_inches='tight')
            plt.close()
        else:
            self.get_logger().warn('No relative position data available for plotting.')

        self.get_logger().info('Plotting complete. Images saved to /tmp/tool_relative_plot.png and /tmp/marker_relative_plot.png')
        
        # We can trigger shutdown now
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTestPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()