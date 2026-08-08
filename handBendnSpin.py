import bpy
import socket
import struct
import math
import time
import mathutils

UDP_IP = "127.0.0.1"
UDP_PORT = 5052
NUM_FLOATS = 21 * 3

ARMATURE_NAME = "Armature"

# Arm = base bone, untouched. Palm = the bone we're rotating.
BONE_NAME = "Palm"

# --- Orientation from landmark geometry ---
# Instead of two independent 1D heuristics (which cross-contaminate each
# other), build an orthonormal basis from the hand landmarks each frame and
# turn that into a real rotation matrix. Pitch/roll/yaw all fall out of the
# same decomposition instead of fighting over the same underlying motion.
#
# Axis mapping from the computed (local hand-basis) rotation into the bone's
# rotation_euler. Same trial-and-error idea as before: if an axis looks
# wrong or inverted, change the index (0/1/2) or negate it below.
EULER_ORDER = 'XYZ'
# computed[0] = rotation about the across-palm axis  -> true pitch  -> bone Z
# computed[1] = rotation about the long (wrist->finger) axis -> roll -> bone Y
# computed[2] = rotation about the palm-normal axis -> yaw/wave -> unused (0 sign)
AXIS_MAP = [2, 1, 0]     # which bone axis each computed euler component drives
AXIS_SIGN = [-1, 1, 0]    # yaw disabled; flip to -1 per axis if a direction is backwards

# Press 'C' while the operator is running to re-capture the rest pose
# (hold your hand in a neutral, flat-facing-camera position when you do).


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def remap(value, in_min, in_max, out_min, out_max):
    t = clamp((value - in_min) / (in_max - in_min), 0.0, 1.0)
    return out_min + t * (out_max - out_min)


def hand_basis_matrix(landmarks):
    """Build an orthonormal rotation matrix from wrist/MCP landmarks."""
    wrist = mathutils.Vector(landmarks[0])
    index_mcp = mathutils.Vector(landmarks[5])
    middle_mcp = mathutils.Vector(landmarks[9])
    pinky_mcp = mathutils.Vector(landmarks[17])

    y_axis = (middle_mcp - wrist).normalized()          # long axis, wrist -> fingers
    v1 = (index_mcp - wrist)
    v2 = (pinky_mcp - wrist)
    z_axis = v1.cross(v2).normalized()                   # palm normal
    x_axis = y_axis.cross(z_axis).normalized()            # across the palm
    z_axis = x_axis.cross(y_axis).normalized()            # re-orthogonalize

    # Columns = basis axes expressed in landmark space
    return mathutils.Matrix((x_axis, y_axis, z_axis)).transposed()


def remove_old_constraints(armature):
    # Cleans up constraints left over from earlier target-based attempts --
    # if those are still on the bone they'll fight the rotation we set here.
    bone = armature.pose.bones.get(BONE_NAME)
    if bone is None:
        return
    for name in ("WristDampedTrack", "WristLockedTrack"):
        con = bone.constraints.get(name)
        if con is not None:
            bone.constraints.remove(con)
            print(f"Removed leftover constraint '{name}'")


class LandmarkReceiver(bpy.types.Operator):
    bl_idname = "wm.landmark_receiver"
    bl_label = "Landmark Receiver"

    _timer = None
    _sock = None
    _start_time = None
    _rest_matrix = None

    def modal(self, context, event):
        if event.type == 'TIMER':
            self.poll_socket(context)

        if event.type == 'C' and event.value == 'PRESS':
            self._rest_matrix = None
            print("Rest pose cleared -- will recalibrate on next frame.")

        if event.type == 'ESC':
            self.cancel(context)
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def update_wrist_rotation(self, armature, landmarks):
        bone = armature.pose.bones.get(BONE_NAME)
        if bone is None:
            return

        current_matrix = hand_basis_matrix(landmarks)

        if self._rest_matrix is None:
            self._rest_matrix = current_matrix
            print("Calibrated rest pose.")
            return

        relative = self._rest_matrix.inverted() @ current_matrix
        euler = relative.to_euler(EULER_ORDER)
        computed = [euler[0], euler[1], euler[2]]
        print(f"ORIENTATION euler={[round(math.degrees(a), 1) for a in computed]}")

        bone.rotation_mode = EULER_ORDER
        rotation = [0.0, 0.0, 0.0]
        for i in range(3):
            rotation[AXIS_MAP[i]] = computed[i] * AXIS_SIGN[i]
        bone.rotation_euler = tuple(rotation)

    def poll_socket(self, context):
        try:
            data, _addr = self._sock.recvfrom(4096)
        except BlockingIOError:
            return

        flat = struct.unpack(f"{NUM_FLOATS}f", data)
        landmarks = [
            (flat[i * 3], flat[i * 3 + 1], flat[i * 3 + 2])
            for i in range(21)
        ]

        armature = bpy.data.objects.get(ARMATURE_NAME)
        if armature is None:
            print(f"Armature '{ARMATURE_NAME}' not found")
            return

        self.update_wrist_rotation(armature, landmarks)

    def execute(self, context):
        armature = bpy.data.objects.get(ARMATURE_NAME)
        if armature is None:
            print(f"Armature '{ARMATURE_NAME}' not found")
            return {'CANCELLED'}

        remove_old_constraints(armature)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((UDP_IP, UDP_PORT))
        self._sock.setblocking(False)

        self._start_time = time.time()

        wm = context.window_manager
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
    bpy.ops.wm.landmark_receiver()