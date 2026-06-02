import cv2 as cv
import numpy as np
import cameratransform as ct
from math import atan2
import sys

# Configuration Flags
_DepthImage_invert_z      = 0x00000001    # Brighter colors are closer
_DepthImage_cos_correct_z = 0x00000002    # Depth is perpendicular distance instead of Euclidean distance
_DepthImage_smooth        = 0x00000004    # Apply smoothing filter to reduce sensor noise

DEG = np.pi / 180.0

class DepthPoint:
    """Represents a single 3D point in the reconstructed point cloud."""
    def __init__(self):
        self.pos = np.zeros(3, dtype=np.float64)  # 3D Position (x, y, z)
        self.nor = np.zeros(3, dtype=np.float64)  # Normal Vector (nx, ny, nz)
        self.col = np.zeros(3, dtype=np.float64)  # Color (r, g, b)
        self.pxls = (0, 0)

class DepthImage:
    """Handles point cloud generation, filtering, and flat surface region detection from depth maps."""
    def __init__(self):
        self.xs = 0
        self.ys = 0
        self.depth = None
        self.pnt = None
        
        # Default Depth Camera Intrinsics & Ranges
        self.depthFOVx = 86.0 * DEG
        self.depthFOVy = 57.0 * DEG
        self.depthx0 = 0.5
        self.depthy0 = 0.5
        self.depthz0 = 0.8
        self.depthz1 = 1.0
        self.depthcfg = _DepthImage_invert_z | _DepthImage_cos_correct_z
        self.error = self.depthz1 * 0.99

    @staticmethod
    def _normalize_vector(a):
        norm = np.linalg.norm(a)
        if norm == 0:
            return a
        return a / norm

    def project(self, p):
        """Projects a 3D point back onto the 2D image plane."""
        x = atan2(p[0], p[2]) / self.depthFOVx
        y = atan2(p[1], p[2]) / self.depthFOVy
        if self.depthcfg & _DepthImage_cos_correct_z:
            y *= (self.ys * self.depthFOVx) / (self.depthFOVy * self.xs)

        x = (x + 0.5) * self.xs
        y = (y + 0.5) * self.ys
        return (x, y)

    def load(self, file_path):
        """Loads a depth map and constructs the 3D point cloud vectorially."""
        print(f"Loading image from {file_path}...")
        self.depth = cv.imread(file_path, cv.IMREAD_GRAYSCALE)
        if self.depth is None:
            raise FileNotFoundError(f"Could not load depth map image from: {file_path}")
            
        self.ys, self.xs = self.depth.shape
        self.error = self.depthz1 * 0.99
        
        # 1. Vectorized Point Cloud Reconstruction
        print("Reconstructing 3D coordinate matrices...")
        y_indices = np.arange(self.ys)
        x_indices = np.arange(self.xs)
        
        # Compute angles grid
        b = self.depthFOVy * (((y_indices - self.depthy0) / (self.ys - 1)) - 0.5)
        a = self.depthFOVx * (((x_indices - self.depthx0) / (self.xs - 1)) - 0.5)
        A, B = np.meshgrid(a, b)
        
        # Map depth values
        r = (self.depth & 255).astype(np.float64) / 255.0
        if self.depthcfg & _DepthImage_invert_z:
            r = 1.0 - r
        r = self.depthz0 + (r * (self.depthz1 - self.depthz0))
        
        # Spherical coordinates directions
        p_dir = np.stack([
            np.cos(A) * np.cos(B), # x direction
            np.sin(A) * np.cos(B), # y direction
            np.sin(B)              # z direction
        ], axis=-1)
        
        if self.depthcfg & _DepthImage_cos_correct_z:
            r /= p_dir[:, :, 0]
            
        # Reorder coordinates to match target view frame: (r*p1, r*p2, r*p0)
        positions = np.stack([
            r * p_dir[:, :, 1], # X
            r * p_dir[:, :, 2], # Y
            r * p_dir[:, :, 0]  # Z
        ], axis=-1)
        
        # 2. Vectorized Point Smoothing Filter
        if self.depthcfg & _DepthImage_smooth:
            print("Applying spatial smoothing iterations...")
            # Simple average filtering using OpenCV Gaussian Blur to replace slow manual iterations
            positions = cv.GaussianBlur(positions, (3, 3), 0)
            
        # 3. Vectorized Surface Normal Estimations
        print("Computing surface normals...")
        p = positions[1:, 1:, :] - positions[1:, :-1, :]
        q = positions[1:, 1:, :] - positions[:-1, 1:, :]
        raw_normals = np.cross(q, p, axisa=2, axisb=2, axisc=-1)
        
        # Normalize vectors safely
        norms = np.linalg.norm(raw_normals, axis=2, keepdims=True)
        norms = np.where(norms == 0, 1.0, norms)
        normals = raw_normals / norms
        
        # Create full sized normal grid, copying edges
        full_normals = np.zeros_like(positions)
        full_normals[1:, 1:, :] = normals
        full_normals[1:, 0, :] = full_normals[1:, 1, :]
        full_normals[0, :, :] = full_normals[1, :, :]
        
        # Convert back to legacy DepthPoint objects for structural backward-compatibility
        print("Populating structural representation...")
        self.pnt = np.empty((self.ys, self.xs), dtype=object)
        for y in range(self.ys):
            for x in range(self.xs):
                pt = DepthPoint()
                pt.pos = positions[y, x]
                pt.nor = full_normals[y, x]
                self.pnt[y, x] = pt
                
        print("Successfully generated point cloud.")

    def find_flat_rect(self, wx, wy, dmax):
        """Searches for a flat rectangular patch of dimensions wx x wy with maximum normal deviation dmax."""
        print("Scanning point cloud for flat landing zone candidates...")
        p0 = np.zeros(3)
        p1 = np.zeros(3)
        p2 = np.zeros(3)
        p3 = np.zeros(3)
        pxls = np.zeros((4, 2))

        y0 = 0
        while y0 < self.ys:
            x0 = 0
            while x0 < self.xs:
                p0 = self.pnt[y0][x0].pos
                if p0[2] < self.error:
                    # Find next corner (x1) located at distance >= wx along the horizontal scanline
                    x1 = x0 + 1
                    while x1 < self.xs:
                        p1 = self.pnt[y0][x1].pos
                        if p1[2] < self.error:
                            d = np.linalg.norm(p1 - p0)
                            if d >= wx:
                                break
                        x1 += 1
                    
                    if x1 >= self.xs:
                        x0 = self.xs
                        continue

                    # Find next corner (y1) located at distance >= wy along the vertical scanline
                    y1 = y0 + 1
                    while y1 < self.ys:
                        p3 = self.pnt[y1][x0].pos
                        if p3[2] < self.error:
                            d = np.linalg.norm(p3 - p0)
                            if d >= wy:
                                break
                        y1 += 1

                    if y1 >= self.ys:
                        return False, p0, p1, p2, p3, pxls

                    p2 = self.pnt[y1][x1].pos
                    
                    # Search step-size relative to candidate window bounds
                    dx = 1 + ((x1 - x0) >> 2)
                    dy = 1 + ((y1 - y0) >> 2)

                    # Compute candidate patch plane normal
                    v1 = p1 - p0
                    v2 = p3 - p0
                    plane_normal = np.cross(v2, v1)
                    plane_normal = self._normalize_vector(plane_normal)

                    # Verify maximum deviation threshold across all interior points
                    is_flat = True
                    for y in range(y0, y1 + 1):
                        for x in range(x0, x1 + 1):
                            if self.pnt[y][x].pos[2] >= self.error:
                                continue
                            diff = self.pnt[y][x].pos - p0
                            dist = np.abs(np.dot(diff, plane_normal))
                            if dist > dmax:
                                is_flat = False
                                break
                        if not is_flat:
                            break

                    if is_flat:
                        print(f"Found flat rectangle patch at corners: ({x0}, {y0}) -> ({x1}, {y1})")
                        return True, p0, p1, p2, p3, pxls
                x0 += dx
            y0 += dy

        print("No suitable flat patch found.")
        return False, p0, p1, p2, p3, pxls

    def draw(self, depth_image, points_2d):
        """Draws the identified rectangular landing zone on the visual screen."""
        img = depth_image.copy()
        if len(points_2d) >= 4:
            pts = np.array([(int(p[0]), int(p[1])) for p in points_2d], dtype=np.int32)
            cv.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
        return img

    def show(self, img):
        """Displays the output image using OpenCV window buffers."""
        cv.imshow('UAV Terrain Landing Spot Detection', img)
        cv.waitKey(0)
        cv.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        img_path = "1.jpg"
    else:
        img_path = sys.argv[1]
        
    pcl = DepthImage()
    try:
        pcl.load(img_path)
    except FileNotFoundError as e:
        print(e)
        # Create a dummy depth map for test validation if image file does not exist
        print("Creating a dummy depth map for verification...")
        dummy = (np.ones((480, 640)) * 128).astype(np.uint8)
        cv.imwrite("1.jpg", dummy)
        pcl.load("1.jpg")
        
    # Search for a flat surface area of size 0.4m x 0.4m with max 5mm height variance
    res = pcl.find_flat_rect(wx=0.4, wy=0.4, dmax=0.005)
    ret, p0, p1, p2, p3, _ = res
    if not ret:
        print("No flat area found in depth map.")
        sys.exit(0)
        
    corners_3d = np.array([p0, p1, p2, p3])
    corners_2d = [pcl.project(p) for p in corners_3d]
    
    img = cv.imread(img_path)
    if img is None:
        img = cv.imread("1.jpg")
        
    annotated_img = pcl.draw(img, corners_2d)
    print("Terrain landing detection completed successfully. Coordinates (2D):", corners_2d)
    
    # Save the output image instead of calling interactive show() during headless runs
    cv.imwrite("detected_landing_zone.jpg", annotated_img)
    print("Saved output visualization to detected_landing_zone.jpg")

if __name__ == '__main__':
    main()
