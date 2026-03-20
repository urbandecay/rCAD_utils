import bpy
from mathutils import Vector
from ..orientation_utils import orthonormal_basis_from_normal

class CAD_BaseTool:
    def __init__(self, core):
        self.core = core
        self.state = core.state
    
    def update(self, context, event, snap_point, snap_normal):
        """Called every mouse move. Handles main logic."""
        pass
        
    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        """Called on Left Click / Enter. Returns 'NEXT_STAGE', 'FINISHED' or None."""
        return None
        
    def handle_input(self, context, event):
        """Called on key press. Returns True if consumed."""
        return False
        
    def cancel(self, context):
        """Cleanup if user cancels."""
        pass

class SurfaceDrawTool(CAD_BaseTool):
    """
    The 'Prep Chef' for any tool that starts by finding a surface point.
    Handles:
    1. Finding the drawing plane (Xp, Yp, Zp) based on surface normal.
    2. Setting the Pivot Point (Stage 0).
    3. Locking the Plane (L Key).
    """
    def __init__(self, core):
        super().__init__(core)
        self.stage = 0
        self.pivot = None  # The first click (Center for Circle, P1 for Line)
        
        # Coordinate System
        self.Xp = None
        self.Yp = None
        self.Zp = None
        
        # Ensure state keys exist
        if "locked" not in self.state: self.state["locked"] = False
        if "locked_normal" not in self.state: self.state["locked_normal"] = None
        if "locked_plane_point" not in self.state: self.state["locked_plane_point"] = None
        if "is_perpendicular" not in self.state: self.state["is_perpendicular"] = False

    def update_initial_plane(self, context, event, snap_point, snap_normal):
        """
        Run this during Stage 0 to snap the compass to walls/floors.
        """
        # If locked, only update the coordinate basis if the normal hasn't been set yet
        if self.state.get("locked") and self.state.get("locked_normal"):
            self.Zp = self.state["locked_normal"]
            self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)
            # We don't return early here anymore; we let it fall through 
            # to keep self.pivot/position updated for the preview.

        # Default to Up if we are floating in void
        if snap_normal is None: 
            snap_normal = Vector((0, 0, 1))
            
        if not self.state.get("locked"):
            # Set the basis from surface if not locked
            self.Zp = snap_normal
            self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)

    def handle_plane_lock_input(self, context, event):
        """
        Handles 'L' key for locking/unlocking the drawing plane.
        Returns True if consumed.
        """
        if event.type == 'L' and event.value == 'PRESS':
            if self.state.get("locked"):
                # Unlock
                self.state["locked"] = False
                self.state["locked_normal"] = None
                self.state["locked_plane_point"] = None
                self.core.report({'INFO'}, "Unlocked")
            else:
                # Lock
                if self.Zp:
                    self.state["locked"] = True
                    self.state["locked_normal"] = self.Zp
                    
                    # --- FIX: Capture the current surface point ---
                    # This prevents the plane from jumping to the origin (0,0,0)
                    if "last_surface_hit" in self.state and self.state["last_surface_hit"]:
                        self.state["locked_plane_point"] = self.state["last_surface_hit"]
                    else:
                        # Fallback if in void
                        self.state["locked_plane_point"] = Vector((0,0,0))
                        
                    self.core.report({'INFO'}, "Locked to Plane")
                else:
                    self.core.report({'WARNING'}, "No Normal to Lock To")
            return True
        return False