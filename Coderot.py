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

# As the hand rotates edge-on to the camera, MCP5 (index) and MCP2 (thumb)
# get closer together in the landmark data. We use that shrinking distance
# as a single rotation DOF around the bone's long axis (local Y).
ROT_AXIS = 1  # 0=X, 1=Y, 2=Z -- local Y is the long axis elsewhere in this rig

# Calibrate these: watch the printed DIST value while rotating your hand to
# each extreme (flat facing camera vs. rotated edge-on) and set accordingly.
DIST_MIN = 0.036   # distance when the hand is rotated edge-on (landmarks closest)
DIST_MAX = 0.22   # distance when the hand is flat, facing the camera (farthest)

# Rotation range mapped onto that distance range, in radians. If the rotation
# direction is backwards, swap ROT_MIN and ROT_MAX.
ROT_MIN = 1.57   # ~ -90 degrees
ROT_MAX = -1.57    # ~ +90 degrees


def distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def remap(value, in_min, in_max, out_min, out_max):
    t = clamp((value - in_min) / (in_max - in_min), 0.0, 1.0)
    return out_min + t * (out_max - out_min)


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

    def modal(self, context, event):
        if event.type == 'TIMER':
            self.poll_socket(context)

        if event.type == 'ESC':
            self.cancel(context)
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def update_wrist_rotation(self, armature, landmarks):
        bone = armature.pose.bones.get(BONE_NAME)
        if bone is None:
            return

        dist = distance(landmarks[5], landmarks[2])
        print(f"WRIST MCP5-MCP2 dist={dist:.4f}")

        angle = remap(dist, DIST_MIN, DIST_MAX, ROT_MIN, ROT_MAX)

        bone.rotation_mode = 'XYZ'
        rotation = [0.0, 0.0, 0.0]
        rotation[ROT_AXIS] = angle
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