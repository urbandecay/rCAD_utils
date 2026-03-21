# 🐛 The Case of the Triple-Splitting Tiler

Yo! So here's the lowdown on why the Mesh Tiler was acting like a neurotic chef and slicing your models in all the wrong places.

## 1. The "Upper Split" (A.K.A. The Teleporting Knife)
**The Symptom:** You saw a cut right at the top boundary of the intersection, like the mesh was being shaved instead of split.
**The Cause:** The `bisect` tool is a bit of a diva—it only speaks "Local Space." We were feeding it "World Space" coordinates.
**The Fix:** 
> "Hey Bisect, cut at Z=5."
> *Mesh at Z=5:* "Okay, cutting 5 units above my center... which is Z=10." 
> 💥 **Wrong.**

We now politely translate the world coordinates into the object's local VIP lounge (Local Space) before asking for a cut. We also fixed the normal vector math so the knife doesn't tilt when the object is stretched.

## 2. The "Double Vision" (Overlapping Geometry)
**The Symptom:** You saw *two* meshes overlapping instead of one clean seam.
**The Cause:** We told Blender to cut the mesh but didn't tell it to throw away the scraps. So the original mesh (the "overlap") was still hanging out at the party, uninvited.
**The Fix:** We flipped the `clear_inner` and `clear_outer` switches. Now, when we cut, we toss the excess geometry into the void. Clean seams only.

## 3. The "Lower Split" (The Gremlin Geometry)
**The Symptom:** A random third cut appeared at the bottom.
**The Cause:** Tiny, invisible bits of geometry (loose vertices, stray faces, 3D dust bunnies) were being detected as "islands." The script thought, "Aha! An intersection!" and tried to slice them, creating ghost planes.
**The Fix:** We installed a bouncer.
- **Rule 1:** You must have at least 4 faces to enter.
- **Rule 2:** You must be at least 10% the size of the biggest guy in the room.
- **Result:** Tiny debris gets kicked out. No more ghost cuts.

---
**Status:** Fixed. The Tiler now cuts exactly once, exactly where you want it. 🔪✨
