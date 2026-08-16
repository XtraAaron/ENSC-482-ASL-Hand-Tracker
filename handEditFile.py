import bpy # Blender Python API
import socket # Used to open UDP socket
import struct # Packs/unpacks raw bytes do and from python values
import math # Math stuff
import time # Time stuff
import mathutils # Vector math stuff
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

# Thumb2
THUMB2_CURL_AXIS = 2
THUMB2_CURL_AMPLITUDE = 1.6

# Index1
INDEX1_CURL_AMPLITUDE = 1.6
INDEX1_SPREAD_AMPLITUDE = .5
INDEX1_CURL_SCALE = 1 # Unused, but dont want to remove cuz it works with it

# Index2
INDEX2_CURL_AMPLITUDE = 1.6

# Index3
INDEX3_CURL_AMPLITUDE = 1.6

# Middle1
MIDDLE1_CURL_AMPLITUDE = 1.6
MIDDLE1_SPREAD_AMPLITUDE = .5
MIDDLE1_CURL_SCALE = 1

# Middle2
MIDDLE2_CURL_AMPLITUDE = 1.6

# Middle3
MIDDLE3_CURL_AMPLITUDE = 1.6

# Ring1
RING1_CURL_AMPLITUDE = 1.6
RING1_SPREAD_AMPLITUDE = .3
RING1_CURL_SCALE = 1

# Ring2
RING2_CURL_AMPLITUDE = 1.6

# Ring3
RING3_CURL_AMPLITUDE = 1.6

# Pinky1
PINKY1_CURL_AMPLITUDE = 1.6
PINKY1_SPREAD_AMPLITUDE = .3
PINKY1_CURL_SCALE = 1

# Pinky2
PINKY2_CURL_AMPLITUDE = 1.6

# Pinky
PINKY3_CURL_AMPLITUDE = 1.6

PALM_BONE_NAME = "Palm" # Bone name palm

EULER_ORDER = 'XYZ' # Defined how blender should accept the Euler Rotation

# Compuete = [pitch, roll, yaw]
# Pitch is rotation about the across-palm axis, or 
# Like the pitch of the plane, or a shoo motion

# Roll is rotation about the long (wrist->finger) axis
# Its like the roll of a plane

# Yaw is rotation about the palm-normal axis which isnt used so we dont care
# Its like waving your hand to say bye, or the yaw motion

# Literally if you have ur fingers pointed forwards, its the same sorta deal as a plane 
PALM_AXIS_MAP = [2, 1, 0] # Defines the bone components each euler drives, so pitch goes to z, roll to y, and yaw to x
PALM_AXIS_SIGN = [-1, 1, 0] # Defines the direction of the axis

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


def hand_basis_matrix(landmarks): # Takes in the 21 landmark triplets
    wrist = mathutils.Vector(landmarks[0]) 
    index_mcp = mathutils.Vector(landmarks[5])
    middle_mcp = mathutils.Vector(landmarks[9])
    pinky_mcp = mathutils.Vector(landmarks[17])
    # Converts the first index into a math utils vector, since norm and transposed only works on that

    y_axis = (middle_mcp - wrist).normalized() # Creates the hands y-axis (roll axis)
    # Does this by normalizing the vector between MCP3 and the wrist, as its the closest thing to a center line along the hand
    
    v1 = (index_mcp - wrist)
    v2 = (pinky_mcp - wrist)
    # Draw 2 more vectors from wrist to index and pinky base

    z_axis = v1.cross(v2).normalized() # The cross of the 2 givea  new vec perpendicularly 2 both
    # Makes a vector normal the palm (around)
    x_axis = y_axis.cross(z_axis).normalized() # This produces the pitch axis, or the axis that goes from index to pinky
    z_axis = x_axis.cross(y_axis).normalized() # This recalcs z using new y and x

    return mathutils.Matrix((x_axis, y_axis, z_axis)).transposed() # Puts result in a 3x3 vector
    # Transpose is needed as blender expecte axis vectors as columns
# This functions is the function that makes the coordinate system relative to the hand (or the wrist)
# So we can still move fingers even when wrist rotates


def swing_twist(quat, twist_axis):
    twist_axis = twist_axis.normalized() # Ensures the axis pases is in unit lenth to be compatible with the math
    qv = mathutils.Vector((quat.x, quat.y, quat.z)) # Gets the imaginary part of the rotation
    # So uh quaternions are complex and basically xyz are the imaginary parts and w is the real part
    # Its a method of representing rotations in 3D space
    proj = twist_axis * qv.dot(twist_axis) # Projects qv onto the twist axis
    # This gives the component of the rotation actually about the axis
    twist = mathutils.Quaternion((quat.w, proj.x, proj.y, proj.z)) # This makes a new quatnarion from the projected vector and og w
    if twist.magnitude < 1e-9:
        twist = mathutils.Quaternion((1, 0, 0, 0)) # Twist becoimes the identity quaternion (no rotation)
        # A check for when the projection is ~0, in this case almost pure pitch and no roll, avoids dividing by 0 when normalizing
    else:
        twist.normalize() # Otherwise normalize as usual
    swing = quat @ twist.inverted() # Remove twist from og rotation, leaving only swing
    return swing, twist # Return
# This function takes in a quaternion (W,X,Y,Z) rotation, and splits it into the pitch and the roll movement


class LandmarkReceiver(bpy.types.Operator):
    # Like a class, specifically a child class taking stuff from parent class bpy.types.Operator
    bl_idname = "wm.landmark_receiver" # Blender operator name
    bl_label = "Landmark Receiver" # The label for our (human/usage in ui) reading

    _timer = None # Stores the reference to the running timer blender makes
    _sock = None # Holds the udp socket object
    # _start_time = None
    _rest_matrix = None  # Stored calibrated wrist rest-pose matrix
    # Pressing "C" forces reset, allowing for recalibration - IMPORTANT!!!!!
    # Likes to go out of alignment if left running too long, this helps fix

    # Class var creation

    def modal(self, context, event):
        if event.type == 'TIMER': # Is this event a timer tick, or the default case
            self.poll_socket(context) # Check UDP for new data

        if event.type == 'C' and event.value == 'PRESS': # If user presses C
            self._rest_matrix = None # Clears rest pose
            print("Wrist rest pose cleared -- will recalibrate on next frame.")

        if event.type == 'ESC': # Checks for esc
            self.cancel(context) # Calls cleanup and stuff
            return {'CANCELLED'} # Tells blender its done

        return {'PASS_THROUGH'} # Returned for evey non-esc event
    # This function gets called for every events and decides what to do depending on the event type

    def update_wrist_rotation(self, armature, landmarks):
        bone = armature.pose.bones.get(PALM_BONE_NAME) # Gets the palm bone name from armature pose bones
        if bone is None:
            return
        # If bone doesnt exist return instead of imploding

        current_matrix = hand_basis_matrix(landmarks) # Calls the basis axis builder

        if self._rest_matrix is None: # Checks if calibration has hapened yet
            self._rest_matrix = current_matrix # If not calibrated, this frame is used as the reference frame
            print("Calibrated wrist rest pose.")
            return

        relative = self._rest_matrix.inverted() @ current_matrix # Computes how much the wrist has moved since calibration
        relative_quat = relative.to_quaternion() # Converts that relative rotation to quaternion vector

        swing, twist = swing_twist(relative_quat, mathutils.Vector((0, 1, 0))) # Gets wrist pitch (swing) and twist (roll)

        roll = twist.angle if twist.axis.y >= 0 else -twist.angle # Gets signed roll angle from twist vector
        # Checks if twist axis points along +Y or -Y and flips sign for consistancy
        pitch = swing.to_euler(EULER_ORDER)[0] # Converts pitch (swing) to Euler and takes X has the pitch
        computed = [pitch, roll, 0.0] # Puts the computed angles + placeholder for yaw
        # --NEED TO ADD YAW FOR J TO WORK--

        bone.rotation_mode = EULER_ORDER # Set bone to use Euler (makes more sense to me so i choose it)
        rotation = [0.0, 0.0, 0.0] # Initalzie rotation array
        for i in range(3): # i goes from 0,1,2
            # 0 is pitch, this gets negated and written into Z rotation
            # 1 is roll, this doesnt get negated and gets writen into Y rotation
            # 2 is yaw, goes into X rotation
            rotation[PALM_AXIS_MAP[i]] = computed[i] * PALM_AXIS_SIGN[i]
            # This here maps each computed value to the bone axis that should be driven. 
        bone.rotation_euler = tuple(rotation) # Apply final rotation
    # This function calculates the wrist rotations relative to the calibrated rest pose

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
# This works with the knuckle, or the base "joint" of ur finger (from MCP to PIP)
# Similar to wrist, it gets the rotations and applies them to said joints

    def apply_thumb1_joint(self, armature, landmarks):
        bone = armature.pose.bones.get("Thumb1")
        if bone is None:
            print("Bone 'Thumb1' not found")
            return

        basis = hand_basis_matrix(landmarks)
        across_axis = basis.col[0]   # index-to-pinky direction
        normal_axis = basis.col[2]   # palm normal

        thumb1_vec = vector(landmarks[2], landmarks[3])  # MCP1 -> IP1

        angle_x = angle_between(thumb1_vec, across_axis)
        angle_z = angle_between(thumb1_vec, normal_axis)

        bone.rotation_mode = 'XYZ'
        rotation = [0.0, 0.0, 0.0]
        rotation[0] = (angle_x - 1.6) * 1
        rotation[1] = 0.0
        rotation[2] = (angle_z - 1.6) * 1
        bone.rotation_euler = tuple(rotation)
        
        
    def apply_thumb2_joint(self, armature, landmarks):
        bone = armature.pose.bones.get("Thumb2")
        if bone is None:
            print("Bone 'Thumb2' not found")
            return

        bend = joint_bend(landmarks, 1, 2, 4)
        curl = ((math.pi / 2) - clamp(-bend, -THUMB2_CURL_AMPLITUDE, 0.0) - 1.6) * 1.2

        bone.rotation_mode = 'XYZ'
        rotation = [curl, 0.0, 0.0]
        bone.rotation_euler = tuple(rotation)


    def apply_curl_joint(self, armature, bone_name, landmarks, idx_a, idx_b, idx_c, curl_amplitude,
                          curl_axis=CURL_AXIS):
        bone = armature.pose.bones.get(bone_name)
        if bone is None:
            print(f"Bone '{bone_name}' not found")
            return

        bend = joint_bend(landmarks, idx_a, idx_b, idx_c)
        curl = clamp(-bend, -curl_amplitude, 0.0)

        bone.rotation_mode = 'XYZ'
        rotation = [0.0, 0.0, 0.0]
        rotation[curl_axis] = curl
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

        # --- Wrist/Palm ---
        self.update_wrist_rotation(armature, landmarks)

        # --- Thumb ---
        self.apply_thumb1_joint(armature, landmarks)
        self.apply_thumb2_joint(armature, landmarks)

        # --- Fingers ---
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
        armature = bpy.data.objects.get(ARMATURE_NAME)
        if armature is None:
            print(f"Armature '{ARMATURE_NAME}' not found")
            return {'CANCELLED'}

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
# Registers the above class with blender, letting it be called via bpy.ops.wm.landmark_receiver()

def unregister():
    bpy.utils.unregister_class(LandmarkReceiver)
# Undoes the previous, cleanup step

if __name__ == "__main__":
    register()
    bpy.ops.wm.landmark_receiver()
# Calls register then runs the operator, starting the lister loop