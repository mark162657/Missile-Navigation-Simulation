"""
High-performance 2D DEM visualization using Fastplotlib (WebGPU-based).
Displays terrain as a top-down heatmap with optional path overlay.

Controls:
    - Mouse drag: Pan view
    - Mouse wheel: Zoom in/out
    - R: Reset view
"""

import sys
import numpy as np
from pathlib import Path
import fastplotlib as fpl

from pathfinder.pathfinding_backend import Pathfinding

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from ..src.terrain.dem_loader import DEMLoader


class Fastplotlib2DDEMViewer:
    """Fastplotlib-based 2D terrain viewer with pathfinding overlay."""
    
    def __init__(self, dem_path: Path, downsample: int = 10, max_pixels: int = 5_000_000):
        """
        Initialize the 2D DEM viewer.
        
        Args:
            dem_path: Path to DEM file
            downsample: Initial downsample factor (1 = full res, 2 = half, 4 = quarter, etc.)
            max_pixels: Maximum number of pixels to prevent slowdown (default: 5 million)
        """
        print("\n" + "="*60)
        print("INITIALIZING FASTPLOTLIB 2D DEM VIEWER")
        print("="*60)
        
        self.dem_loader = DEMLoader(dem_path)
        
        # Load DEM data
        print(f"\nLoading DEM: {dem_path.name}")
        print(f"Original shape: {self.dem_loader.shape} ({self.dem_loader.shape[0] * self.dem_loader.shape[1]:,} pixels)")
        
        # Auto-adjust downsample to stay within pixel limit
        original_pixels = self.dem_loader.shape[0] * self.dem_loader.shape[1]
        self.downsample = downsample
        
        while True:
            downsampled_pixels = (self.dem_loader.shape[0] // self.downsample) * (self.dem_loader.shape[1] // self.downsample)
            if downsampled_pixels <= max_pixels:
                break
            self.downsample += 1
        
        if self.downsample != downsample:
            print(f"WARNING: Auto-adjusted downsample from {downsample} to {self.downsample} to stay within {max_pixels:,} pixel limit")
        
        # Downsample for performance
        self.dem_data = self.dem_loader.data[::self.downsample, ::self.downsample].astype(np.float32)
        self.dem_data[self.dem_data <= -100] = np.nan  # Remove nodata
        
        print(f"Viewing shape: {self.dem_data.shape} ({self.dem_data.size:,} pixels)")
        print(f"Reduction: {original_pixels / self.dem_data.size:.1f}x")
        
        # Normalize elevation
        valid_data = self.dem_data[~np.isnan(self.dem_data)]
        self.elev_min = np.percentile(valid_data, 2)
        self.elev_max = np.percentile(valid_data, 98)
        self.elev_range = self.elev_max - self.elev_min
        
        print(f"Elevation range: {self.elev_min:.0f}m - {self.elev_max:.0f}m")
        
        # Create figure
        self.figure = fpl.Figure(size=(1400, 900))
        
        # Path data
        self.path = None
        self.path_graphic = None
        
        # Create terrain heatmap
        self._create_terrain_heatmap()
        
        print("\n" + "="*60)
        print("CONTROLS:")
        print("  Mouse Drag: Pan | Wheel: Zoom")
        print("  R: Reset view")
        print("="*60 + "\n")
    
    def _create_terrain_heatmap(self):
        """Create the 2D terrain heatmap using Fastplotlib."""
        print("Generating 2D terrain heatmap...")
        
        # Replace NaN with minimum elevation for visualization
        display_data = np.nan_to_num(self.dem_data, nan=self.elev_min)
        
        # Add heatmap to figure
        self.heatmap = self.figure[0, 0].add_image(
            data=display_data,
            cmap="terrain",
            vmin=self.elev_min,
            vmax=self.elev_max,
        )
        
        # Set title
        self.figure[0, 0].name = f"DEM Terrain (Elevation: {self.elev_min:.0f}m - {self.elev_max:.0f}m)"
        
        # Add hover callback to show coordinates and elevation
        @self.heatmap.add_event_handler("pointer_move")
        def hover_callback(ev):
            # Get pixel coordinates from event
            if hasattr(ev, 'pick_info') and ev.pick_info is not None:
                x_idx = int(ev.pick_info['index'][1])  # column
                y_idx = int(ev.pick_info['index'][0])  # row
                
                # Get elevation at this point
                if 0 <= y_idx < self.dem_data.shape[0] and 0 <= x_idx < self.dem_data.shape[1]:
                    elevation = self.dem_data[y_idx, x_idx]
                    if not np.isnan(elevation):
                        # Convert back to original pixel coordinates
                        orig_row = y_idx * self.downsample
                        orig_col = x_idx * self.downsample
                        
                        # Convert to GPS if possible
                        lat, lon = self.dem_loader.pixel_to_lat_lon(orig_row, orig_col)
                        
                        print(f"\rPixel: ({orig_row}, {orig_col}) | GPS: ({lat:.4f}, {lon:.4f}) | Elevation: {elevation:.1f}m", end='', flush=True)
        
        print("[OK] Terrain heatmap created")
    
    def load_path(self, start_gps: tuple[float, float], end_gps: tuple[float, float], 
                  heuristic_weight: float = 1.5):
        """
        Compute pathfinding and add overlay.
        
        Args:
            start_gps: (lat, lon) start coordinate
            end_gps: (lat, lon) end coordinate
            heuristic_weight: A* heuristic weight
        """
        print("\n" + "="*60)
        print("COMPUTING PATH...")
        print("="*60)
        
        pf = Pathfinding()
        
        # Convert GPS to pixel
        start_row, start_col = pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1])
        end_row, end_col = pf.dem_loader.lat_lon_to_pixel(end_gps[0], end_gps[1])
        
        start_pixel = (start_row, start_col)
        end_pixel = (end_row, end_col)
        
        print(f"Start: {start_gps} -> Pixel {start_pixel}")
        print(f"End:   {end_gps} -> Pixel {end_pixel}")
        
        # Compute path
        path = pf.pathfinding(start_pixel, end_pixel, heuristic_weight=heuristic_weight)
        
        if path:
            self.path = path
            print(f"[OK] Path computed: {len(path)} waypoints")
            self._add_path_overlay()
        else:
            print("[ERROR] No path found")
    
    def _add_path_overlay(self):
        """Add 2D path overlay to the visualization."""
        if not self.path:
            return
        
        # Convert path to downsampled coordinates
        path_2d = []
        for row, col in self.path:
            # Downsample coordinates
            ds_row = row // self.downsample
            ds_col = col // self.downsample
            
            # Check bounds
            if 0 <= ds_row < self.dem_data.shape[0] and 0 <= ds_col < self.dem_data.shape[1]:
                path_2d.append([ds_col, ds_row])  # Note: (x, y) = (col, row)
        
        path_2d = np.array(path_2d, dtype=np.float32)
        
        # Add line to figure
        self.path_graphic = self.figure[0, 0].add_line(
            data=path_2d,
            thickness=3,
            colors="red",
            name="Flight Path"
        )
        
        # Add start and end markers
        if len(path_2d) > 0:
            # Start marker (green)
            self.start_marker = self.figure[0, 0].add_scatter(
                data=path_2d[0:1],
                sizes=15,
                colors="green",
                name="Start"
            )
            
            # End marker (red)
            self.end_marker = self.figure[0, 0].add_scatter(
                data=path_2d[-1:],
                sizes=15,
                colors="red",
                name="End"
            )
        
        print("[OK] Path overlay added")
    
    def show(self):
        """Display the visualization."""
        print("\n>> Starting viewer...")
        print("Press Ctrl+C or close the window to exit...")
        self.figure.show(maintain_aspect=False)
        
        # Keep the window open using rendercanvas
        import rendercanvas.auto
        rendercanvas.auto.loop.run()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Fastplotlib 2D DEM Viewer with Path Overlay')
    parser.add_argument('--downsample', type=int, default=10, 
                        help='Downsample factor (1=full res, 2=half, 4=quarter, etc.)')
    parser.add_argument('--max-pixels', type=int, default=5_000_000,
                        help='Maximum pixels to prevent slowdown (default: 5 million)')
    parser.add_argument('--path', action='store_true', 
                        help='Enable pathfinding overlay')
    parser.add_argument('--start-lat', type=float, default=56.0)
    parser.add_argument('--start-lon', type=float, default=95.0)
    parser.add_argument('--end-lat', type=float, default=56.0)
    parser.add_argument('--end-lon', type=float, default=95.5)
    parser.add_argument('--weight', type=float, default=1.5, 
                        help='A* heuristic weight')
    
    args = parser.parse_args()
    
    # DEM path
    dem_path = Path(__file__).parent / 'data' / 'dem' / 'merged_dem_sib_N54_N59_E090_E100.tif'
    
    if not dem_path.exists():
        print(f"[ERROR] DEM file not found at {dem_path}")
        sys.exit(1)
    
    # Create viewer
    viewer = Fastplotlib2DDEMViewer(dem_path, downsample=args.downsample, max_pixels=args.max_pixels)
    
    # Load path if requested
    if args.path:
        start_gps = (args.start_lat, args.start_lon)
        end_gps = (args.end_lat, args.end_lon)
        viewer.load_path(start_gps, end_gps, heuristic_weight=args.weight)
    
    # Show
    viewer.show()
