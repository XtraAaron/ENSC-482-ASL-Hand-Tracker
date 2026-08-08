import bpy
import socket
import struct
import math
import time
# The import stuff

# UDP stuff, we using udp rn to send data between the py code and the blender stuff
UDP_IP = "127.0.0.1"
UDP_PORT = 5052
NUM_FLOATS = 21 * 3

ARMATURE_NAME = "Armature"
# Under the downloaded rig, the name of the of the bone thing

# Bone Calibriation stuff
# Axis describes which local rotational axis the bone will use. 
# Decided to use 2 types for the fingers, 0 being x, y being 1, and z being 2
# Blender is weird, so i only got it to work with the curl being the z and the spread being the x
# Amplitude is used to clamp the rotation and force it within the desired range
# From my own observation of my fingers, it appears to be from 0 to pi/2 or 0 to ~1.6
# Spread appeared to be ~30 degrees, so set it to .5 (all units taken are in rads)
# Only the base (x1) has spread, as it has 2 Degrees of Freedom. Other joints are single degree
CURL_AXIS = 2 
SPREAD_AXIS = 0

# Note: Index1, 2... Indicate the bone in blender. 1 being the base, and 3 being the tip
# Index1
INDEX1_CURL_AMPLITUDE = 1.6 
INDEX1_SPREAD_AMPLITUDE = .5
INDEX1_CURL_SCALE = 1 # Unused, but dont want to remove cuz it works with it

# Index2
INDEX2_CURL_AMPLITUDE = 1.6

# Index3
INDEX3_CURL_AMPLITUDE = .8

# Middle1
MIDDLE1_CURL_AMPLITUDE = 1.6
MIDDLE1_SPREAD_AMPLITUDE = .5
MIDDLE1_CURL_SCALE = 1

# Middle2
MIDDLE2_CURL_AMPLITUDE = 1.6

# Middle3
MIDDLE3_CURL_AMPLITUDE = .8

# Ring1
RING1_CURL_AMPLITUDE = 1.6
RING1_SPREAD_AMPLITUDE = .3
RING1_CURL_SCALE = 1

# Ring2
RING2_CURL_AMPLITUDE = 1.6

# Ring3
RING3_CURL_AMPLITUDE = .8

# Pinky1
PINKY1_CURL_AMPLITUDE = 1.6
PINKY1_SPREAD_AMPLITUDE = .3
PINKY1_CURL_SCALE = 1

# Pinky2
PINKY2_CURL_AMPLITUDE = 1.6

# Pinky
PINKY3_CURL_AMPLITUDE = .8


def vector(p_from, p_to):
    return (p_to[0] - p_from[0], p_to[1] - p_from[1], p_to[2] - p_from[2])
# This functions makes a 3D vector from one point to another
# Used to make a vector from one landmark point to another
# From and to are vectors themselves


def normalize(v):
    mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2) # Get vector length
    if mag == 0: # 0 check
        return (0.0, 0.0, 0.0)
    return (v[0]/mag, v[1]/mag, v[2]/mag)
    # Else normalize the vector


def cross(v1, v2):
    return (
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0],
    )


def dot(v1, v2):
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
# Cross and dot product, standard formula stuff


def angle_between(v1, v2):
    d = dot(v1, v2)

    mag1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
    # Length stuff
    if mag1 == 0 or mag2 == 0: # Handle 0 to avoid division by 0
        return 0.0
    
    cos_angle = max(-1.0, min(1.0, d / (mag1 * mag2)))
    return math.acos(cos_angle)
# Finds the angle between 2 vectors, using formula cos^{-1}((v1*v2)/(|v1||v2|))


# Landmarks is what comes from the other code
# idx is the index, just which particular landmark we want
def joint_bend(landmarks, idx_a, idx_b, idx_c):
    v1 = vector(landmarks[idx_a], landmarks[idx_b])
    v2 = vector(landmarks[idx_b], landmarks[idx_c])
    return angle_between(v1, v2)
# Calculates the angle between 3 verticies. Always returns a value from 0 to pi


def joint_bend_curl_only(landmarks, idx_a, idx_b, idx_c, side):
    v1 = vector(landmarks[idx_a], landmarks[idx_b])
    v2 = vector(landmarks[idx_b], landmarks[idx_c])
    side_component = dot(v2, side) # Gives us the length of only the particular side we are looking at
    v2_curl = (
        v2[0] - side_component * side[0],
        v2[1] - side_component * side[1],
        v2[2] - side_component * side[2],
    ) 
    # Removes the side component
    return angle_between(v1, v2_curl)
# Similar to joint bend, but remove the side component first
# Side is a vector that points in the direction of spread, needed due to wrist rotation
# By removing this component, we get just a vector just for the curl of the fingers


# Forward and side represent the axis relative to the hand, note may note a unit vector since its relative
# Forward is up (base to tip of finger), side is index to pinky
def spread_angle(finger_vector, forward, side): 
    v_forward = dot(finger_vector, forward) # Forward component (up)
    v_side = dot(finger_vector, side) # Side component (index to pinky)
    return math.atan2(v_side, v_forward)
# Finds the angle of the finger relative to the axis/hand
# In this case, looking straight at ur palm, with fingers facing up, the "spread" of ur fingers


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))
# Restricts value to stay within max and min, used to force stuff to remain within bounds


class LandmarkReceiver(bpy.types.Operator):
    # Like a class, specifically a child class taking stuff from parent class bpy.types.Operator
    bl_idname = "wm.landmark_receiver" # Blender operator name
    bl_label = "Landmark Receiver" # The label for our (human/usage in ui) reading

    _timer = None
    _sock = None
    _start_time = None
    # Class var creation

    def modal(self, context, event):
        if event.type == 'TIMER':
            self.poll_socket(context)

        if event.type == 'ESC':
            self.cancel(context)
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def apply_base_joint(self, armature, bone_name, landmarks, points, forward, side,
                          curl_amplitude, curl_scale, spread_amplitude):
        bone = armature.pose.bones.get(bone_name)
        if bone is None:
            print(f"Bone '{bone_name}' not found")
            return

        bend = joint_bend_curl_only(landmarks, points[0], points[1], points[2], side) * curl_scale
        curl = clamp(-bend, -curl_amplitude, 0.0)

        finger_vec = vector(landmarks[points[1]], landmarks[points[2]])
        spread_raw = spread_angle(finger_vec, forward, side)
        spread = clamp(spread_raw, -spread_amplitude, spread_amplitude)

        bone.rotation_mode = 'XYZ'
        rotation = [0.0, 0.0, 0.0]
        rotation[CURL_AXIS] = curl
        rotation[SPREAD_AXIS] = spread
        bone.rotation_euler = tuple(rotation)

    def apply_curl_joint(self, armature, bone_name, landmarks, idx_a, idx_b, idx_c, curl_amplitude):
        bone = armature.pose.bones.get(bone_name)
        if bone is None:
            print(f"Bone '{bone_name}' not found")
            return

        bend = joint_bend(landmarks, idx_a, idx_b, idx_c)
        curl = clamp(-bend, -curl_amplitude, 0.0)

        bone.rotation_mode = 'XYZ'
        rotation = [0.0, 0.0, 0.0]
        rotation[CURL_AXIS] = curl
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

        forward = normalize(vector(landmarks[0], landmarks[9]))
        palm_normal = normalize(cross(
            vector(landmarks[0], landmarks[5]),
            vector(landmarks[0], landmarks[17]),
        ))
        side = normalize(cross(palm_normal, forward))

        # --- Index ---
        self.apply_base_joint(armature, "Index1", landmarks, (0, 5, 6), forward, side,
                               INDEX1_CURL_AMPLITUDE, INDEX1_CURL_SCALE, INDEX1_SPREAD_AMPLITUDE)
        self.apply_curl_joint(armature, "Index2", landmarks, 5, 6, 7, INDEX2_CURL_AMPLITUDE)
        self.apply_curl_joint(armature, "Index3", landmarks, 6, 7, 8, INDEX3_CURL_AMPLITUDE)

        # --- Middle ---
        self.apply_base_joint(armature, "Middle1", landmarks, (0, 9, 10), forward, side,
                               MIDDLE1_CURL_AMPLITUDE, MIDDLE1_CURL_SCALE, MIDDLE1_SPREAD_AMPLITUDE)
        self.apply_curl_joint(armature, "Middle2", landmarks, 9, 10, 11, MIDDLE2_CURL_AMPLITUDE)
        self.apply_curl_joint(armature, "Middle3", landmarks, 10, 11, 12, MIDDLE3_CURL_AMPLITUDE)

        # --- Ring ---
        self.apply_base_joint(armature, "Ring1", landmarks, (0, 13, 14), forward, side,
                               RING1_CURL_AMPLITUDE, RING1_CURL_SCALE, RING1_SPREAD_AMPLITUDE)
        self.apply_curl_joint(armature, "Ring2", landmarks, 13, 14, 15, RING2_CURL_AMPLITUDE)
        self.apply_curl_joint(armature, "Ring3", landmarks, 14, 15, 16, RING3_CURL_AMPLITUDE)

        # --- Pinky ---
        self.apply_base_joint(armature, "Pinky1", landmarks, (0, 17, 18), forward, side,
                               PINKY1_CURL_AMPLITUDE, PINKY1_CURL_SCALE, PINKY1_SPREAD_AMPLITUDE)
        self.apply_curl_joint(armature, "Pinky2", landmarks, 17, 18, 19, PINKY2_CURL_AMPLITUDE)
        self.apply_curl_joint(armature, "Pinky3", landmarks, 18, 19, 20, PINKY3_CURL_AMPLITUDE)

    def execute(self, context):
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