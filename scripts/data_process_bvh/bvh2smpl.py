from anim import amass
from anim import bvh
from anim.animation import Animation
from util import quat
import numpy as np

def save_as_npz(output_file, smpl_poses, smpl_trans, gender, fps):
    np.savez(output_file, trans=smpl_trans, gender=gender, mocap_framerate=fps, poses=smpl_poses)

# def reverse_animation_to_amass(anim: Animation, skel: Skel, scale: float = 100.0) -> dict:
def reverse_animation_to_amass(output,quats,trans,fps,skel, scale: float = 100.0) -> dict:
    # quats = anim.quats
    # trans = anim.trans
    # fps = anim.fps

    # # Inverse of root quaternion transformation
    # trans_quat_inv = quat.inv(quat.mul(quat.from_angle_axis(np.pi / 2, [1, 0, 0]), quat.from_angle_axis(np.pi / 2, [0, 1, 0])))
    # root_rot_inv = quat.mul(trans_quat_inv[None], quats[:, 0])
    # quats[:, 0] = root_rot_inv

    # # Inverse of root quaternion transformation
    # trans_quat_inv_x = quat.inv(quat.from_angle_axis(-np.pi / 2, [1, 0, 0]))
    # trans_quat_inv_y = quat.inv(quat.from_angle_axis(-np.pi / 2, [0, 1, 0]))
    # trans_quat_inv = quat.mul(trans_quat_inv_y, trans_quat_inv_x)  # Reverse order and apply inverse

    trans_quat_inv_z = quat.inv(quat.from_angle_axis(-np.pi / 2, [0, 0, 1]))
    trans_quat_inv_y = quat.inv(quat.from_angle_axis(-np.pi / 2, [0, 1, 0]))
    trans_quat_inv = quat.mul(trans_quat_inv_y, trans_quat_inv_z)  # Combine inverses
    
    root_rot_inv = quat.mul(trans_quat_inv[None], quats[:, 0])
    quats[:, 0] = root_rot_inv


    # Inverse of translation transformation
    trans = trans @ quat.to_xform(quat.inv(trans_quat_inv)).T
    root_pos = skel.offsets[0][None].repeat(len(quats), axis=0)
    trans = trans - root_pos
    trans = trans / scale

    # Inverse of axis-angle to quaternion conversion
    axangles = quat.to_axis_angle(quats)

    # Inverse of reshaping quaternions to axis-angle
    num_frames = len(axangles)
    axangles = axangles.reshape([num_frames, -1])

    # Collecting the inverse data
    amass_dict = {
        "poses": axangles,
        "trans": trans,
        "mocap_framerate": fps
    }
    gender = 'male'
    save_as_npz(output, axangles, trans, gender, fps)

    return amass_dict

anim_bvh = bvh.load(filepath=r"C:\Users\xxx\Desktop\punch_motions\punch_motions\sample18.bvh",load_skel=True,load_pose=True) #adjust path accordingly for linux systems

print('skel_joints:',anim_bvh.skel.joints[0])
print('quats:',anim_bvh.quats.shape,'trans:',anim_bvh.trans.shape,'fps:',anim_bvh.fps,'anim_name:',anim_bvh.name)

smpl_joint_names = [
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
    ]

    
bvh_to_smpl_map = {
        'Hips': 'pelvis',  #
        'LeftUpLeg': 'left_hip', #
        'RightUpLeg': 'right_hip', #
        'Spine': 'spine1',  #
        'LeftLeg': 'left_knee', #
        'RightLeg': 'right_knee', #
        'Spine1': 'spine2', #
        'LeftFoot': 'left_ankle', #
        'RightFoot': 'right_ankle', #
        'Spine2': 'spine3', #
        'LeftToeBase': 'left_foot', #left_toe
        'RightToeBase': 'right_foot', #right_toe
        'Neck': 'neck', #
        'LeftShoulder': 'left_collar', #
        'RightShoulder': 'right_collar', #
        'LeftArm': 'left_shoulder', #
        'RightArm': 'right_shoulder', #
        'Head': 'head', #
        'LeftForeArm': 'left_elbow', #
        'RightForeArm': 'right_elbow', #
        'LeftHand': 'left_wrist', #
        'RightHand': 'right_wrist', #
        'LeftHandMiddle1': 'left_hand', #
        'RightHandMiddle1': 'right_hand', #
    }

joints_to_use=[]

# for skel_joint in anim_bvh.skel.joints:
#     if skel_joint.name in bvh_to_smpl_map:
#         # joints_to_use.append(smpl_joint_names.index(bvh_to_smpl_map[skel_joint.name]))
#         joints_to_use.append(anim_bvh.skel.joints.index(skel_joint))


for joint_name in smpl_joint_names:
    for skel_joint in anim_bvh.skel.joints:
        if skel_joint.name in bvh_to_smpl_map and bvh_to_smpl_map[skel_joint.name]==joint_name:
            # joints_to_use.append(smpl_joint_names.index(bvh_to_smpl_map[skel_joint.name]))
            joints_to_use.append(anim_bvh.skel.joints.index(skel_joint))


# joints_to_use.sort()
print(joints_to_use,len(joints_to_use))

quats_subset= anim_bvh.quats[:,joints_to_use,:]
print(quats_subset.shape)

output_file= r"C:\Users\xxx\Desktop\bvh_sample_xyz\punch_motions\sample18.npz"
reverse_animation_to_amass(output=output_file,quats=quats_subset,trans=anim_bvh.trans,fps=anim_bvh.fps*30,skel=anim_bvh.skel)
# reverse_animation_to_amass(output=output_file,quats=anim_bvh.quats,trans=anim_bvh.trans,fps=anim_bvh.fps,skel=anim_bvh.skel)




