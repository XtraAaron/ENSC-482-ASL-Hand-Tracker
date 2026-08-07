import bpy
import socket
import struct
import math

# --- Must match the Python camera script exactly ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5052
NUM_FLOATS = 21 * 3  # 21 landmarks * (x, y, z)

# --- Names to change if yours are different ---
ARMATURE_NAME = "Armature"   # the armature object name, from your Outliner

# Base-joint-only for now (wrist -> knuckle -> next joint). Sub-joints
# (Index2/Index3 etc.) come later once the base joints behave correctly.
FINGERS = {
    "Thumb":  {"points": [0, 1, 2],   "bones": ["Thumb1"]},
    "Index":  {"points": [0, 5, 6, 7, 8],   "bones": ["Index1", "Index2", "Index3"]},
    "Middle": {"points": [0, 9, 10, 11, 12], "bones": ["Middle1", "Middle2", "Middle3"]},
    "Ring":   {"points": [0, 13, 14, 15, 16], "bones": ["Ring1", "Ring2", "Ring3"]},
    "Pinky":  {"points": [0, 17, 18, 19, 20], "bones": ["Pinky1", "Pinky2", "Pinky3"]},
}

# --- Per-bone rotation settings ---
# Different bones on the same rig can have different local rotation
# orientations, so one global axis/sign doesn't necessarily work for all
# of them. Tune each bone independently here instead.
#
# axis:   0 = X, 1 = Y, 2 = Z -- which local axis this bone bends around
# sign:   1 or -1 -- flips bend direction
# offset: radians -- added on top of the bend, use this if the bone's
#         rest pose isn't a flat/straight finger, so bend=0 still looks
#         right instead of already curled
#
# Defaults below are placeholders (guesses) -- use print_bone_matrices()
# further down to find the real values for your rig, then fill these in.
BONE_SETTINGS = {
    "Thumb1":  {"axis": 0, "sign": 1,  "offset": 0.0},
    "Index1":  {"axis": 2, "sign": -1, "offset": 0.0},
    "Index2":  {"axis": 2, "sign": -1, "offset": 0.0},
    "Index3":  {"axis": 2, "sign": -1, "offset": 0.0},
    "Middle1": {"axis": 2, "sign": -1, "offset": 0.0},
    "Middle2": {"axis": 2, "sign": -1, "offset": 0.0},
    "Middle3": {"axis": 2, "sign": -1, "offset": 0.0},
    "Ring1":   {"axis": 2, "sign": -1, "offset": 0.0},
    "Ring2":   {"axis": 2, "sign": -1, "offset": 0.0},
    "Ring3":   {"axis": 2, "sign": -1, "offset": 0.0},
    "Pinky1":  {"axis": 2, "sign": -1, "offset": 0.0},
    "Pinky2":  {"axis": 2, "sign": -1, "offset": 0.0},
    "Pinky3":  {"axis": 2, "sign": -1, "offset": 0.0},
}


def vector(p_from, p_to):
    # Simple 3D vector from one landmark point to another (tuples of x,y,z).
    return (p_to[0] - p_from[0], p_to[1] - p_from[1], p_to[2] - p_from[2])


def angle_between(v1, v2):
    # Standard angle-between-two-vectors formula using the dot product:
    # cos(angle) = (v1 . v2) / (|v1| * |v2|)
    dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)

    if mag1 == 0 or mag2 == 0:
        return 0.0

    # Clamp to [-1, 1] to avoid math domain errors from floating point
    # rounding pushing the ratio just barely outside that range.
    cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    return math.acos(cos_angle)  # returns radians


def joint_bend(landmarks, idx_a, idx_b, idx_c):
    # Given three landmark indices forming a joint (a -> b -> c), returns
    # how much that joint is bent, in radians.
    # A straight finger has segments pointing the same direction, so
    # angle_between() returns ~pi (180 degrees) -> bend should be ~0.
    # A fully curled joint returns close to 0 -> bend should be large.
    v1 = vector(landmarks[idx_a], landmarks[idx_b])
    v2 = vector(landmarks[idx_b], landmarks[idx_c])
    straight_angle = angle_between(v1, v2)
    return angle_between(v1, v2)


def finger_bends(landmarks, points):
    # points is a list of landmark indices along one finger's chain.
    # Walk it and compute a bend at each interior joint.
    bends = []
    for i in range(1, len(points) - 1):
        bends.append(joint_bend(landmarks, points[i - 1], points[i], points[i + 1]))
    return bends


def print_bone_matrices():
    """Debug helper -- run this ONCE manually from the Scripting tab
    console (not part of the live tracking loop) to inspect each bone's
    local rotation matrix. Compare this against what you see happen when
    you manually rotate that bone on X/Y/Z in Pose Mode, to figure out
    the real axis/sign/offset values for BONE_SETTINGS.

    Usage, in the Scripting console:
        import blender_receiver
        blender_receiver.print_bone_matrices()
    """
    armature = bpy.data.objects.get(ARMATURE_NAME)
    if armature is None:
        print(f"Armature '{ARMATURE_NAME}' not found")
        return
    for bone in armature.pose.bones:
        print(bone.name)
        print(bone.matrix_basis)
        print("---")


class LandmarkReceiver(bpy.types.Operator):
    """Modal operator: on a timer, poll the UDP socket for new landmark
    data and apply it to the base finger bones using per-bone axis/sign/
    offset settings from BONE_SETTINGS."""

    bl_idname = "wm.landmark_receiver"
    bl_label = "Landmark Receiver"

    _timer = None
    _sock = None

    def modal(self, context, event):
        if event.type == 'TIMER':
            self.poll_socket(context)

        # Press ESC to stop the operator cleanly.
        if event.type == 'ESC':
            self.cancel(context)
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def poll_socket(self, context):
        # Try to read one packet. Since the socket is non-blocking, this
        # raises BlockingIOError if nothing has arrived yet -- that's
        # expected and not an error, just "no new data this tick".
        try:
            data, _addr = self._sock.recvfrom(4096)
        except BlockingIOError:
            return

        # Unpack the 63 floats back out of the binary packet.
        flat = struct.unpack(f"{NUM_FLOATS}f", data)

        # Regroup the flat list of 63 floats into 21 (x, y, z) tuples,
        # so landmarks[5] gives you landmark index 5's (x, y, z) directly.
        landmarks = [
            (flat[i * 3], flat[i * 3 + 1], flat[i * 3 + 2])
            for i in range(21)
        ]

        armature = bpy.data.objects.get(ARMATURE_NAME)
        if armature is None:
            print(f"Armature '{ARMATURE_NAME}' not found")
            return

        for finger_name, finger_data in FINGERS.items():
            bends = finger_bends(landmarks, finger_data["points"])
            bones = finger_data["bones"]

            for bone_name, bend in zip(bones, bends):
                bone = armature.pose.bones.get(bone_name)
                if bone is None:
                    print(f"Bone '{bone_name}' not found")
                    continue

                settings = BONE_SETTINGS.get(bone_name)
                if settings is None:
                    print(f"No BONE_SETTINGS entry for '{bone_name}', skipping")
                    continue

                bone.rotation_mode = 'XYZ'

                rotation = [0.0, 0.0, 0.0]
                rotation[settings["axis"]] = settings["offset"] + settings["sign"] * bend
                bone.rotation_euler = tuple(rotation)

    def execute(self, context):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((UDP_IP, UDP_PORT))
        # Non-blocking so recvfrom() never freezes Blender waiting for data.
        self._sock.setblocking(False)

        wm = context.window_manager
        # Poll every 0.01s (100 times/sec) -- plenty fast relative to
        # camera frame rate, and cheap since each poll is just a socket check.
        self._timer = wm.event_timer_add(0.01, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        if self._sock:
            self._sock.close()
        print("Landmark receiver stopped.")


def register():
    bpy.utils.register_class(LandmarkReceiver)


def unregister():
    bpy.utils.unregister_class(LandmarkReceiver)


if __name__ == "__main__":
    register()
    # Starts the modal operator immediately when you run this script.
    bpy.ops.wm.landmark_receiver()