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
BONE_NAME = "Thumb1"
TARGET_NAME = "Thumb1_Target"  # empty created automatically, don't make this by hand

# If the thumb points the wrong way / mirrored, flip these signs first before anything else
MAP_X = -1.0
MAP_Y = 1.0
MAP_Z = -1.0

TARGET_DISTANCE = 0.3  # how far in front of the bone the empty sits; tune if tracking looks "weak"


def vector(p_from, p_to):
    return (p_to[0] - p_from[0], p_to[1] - p_from[1], p_to[2] - p_from[2])


def normalize(v):
    mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if mag == 0:
        return (0.0, 0.0, 0.0)
    return (v[0]/mag, v[1]/mag, v[2]/mag)


def ensure_target(armature):
    target = bpy.data.objects.get(TARGET_NAME)
    if target is None:
        target = bpy.data.objects.new(TARGET_NAME, None)
        target.empty_display_size = 0.05
        target.empty_display_type = 'SPHERE'
        bpy.context.collection.objects.link(target)
        print(f"Created target empty '{TARGET_NAME}'")
    return target


def ensure_constraint(armature):
    bone = armature.pose.bones.get(BONE_NAME)
    if bone is None:
        print(f"Bone '{BONE_NAME}' not found")
        return

    con = bone.constraints.get("ThumbDampedTrack")
    if con is None:
        con = bone.constraints.new('DAMPED_TRACK')
        con.name = "ThumbDampedTrack"
        con.target = bpy.data.objects.get(TARGET_NAME)
        con.track_axis = 'TRACK_Y'  # bones point along local Y
        print("Added Damped Track constraint to Thumb1")


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

    def update_thumb1_target(self, armature, landmarks):
        bone = armature.pose.bones.get(BONE_NAME)
        target = bpy.data.objects.get(TARGET_NAME)
        if bone is None or target is None:
            return

        raw_dir = vector(landmarks[2], landmarks[3])
        dx, dy, dz = normalize(raw_dir)
        print(f"THUMB dx={dx:.4f} dy={dy:.4f} dz={dz:.4f}")
        dx, dy, dz = dz, dx, dy
        # 2
        dx, dy, dz = dx * MAP_X, dy * MAP_Y, dz * MAP_Z

        bone_head_world = armature.matrix_world @ bone.head

        target.location = (
            bone_head_world.x + dx * TARGET_DISTANCE,
            bone_head_world.y + dy * TARGET_DISTANCE,
            bone_head_world.z + dz * TARGET_DISTANCE,
        )

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

        self.update_thumb1_target(armature, landmarks)

    def execute(self, context):
        armature = bpy.data.objects.get(ARMATURE_NAME)
        if armature is None:
            print(f"Armature '{ARMATURE_NAME}' not found")
            return {'CANCELLED'}

        ensure_target(armature)
        ensure_constraint(armature)

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