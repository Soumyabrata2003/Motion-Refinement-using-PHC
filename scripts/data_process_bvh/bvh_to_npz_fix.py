import os
# Change directory
# os.chdir(r"C:\Users\Soumyabrata\smpl2bvh\CharacterAnimationTools")
# # Verify current directory
# print(os.getcwd())

import sys
sys.path.append(r"C:\Users\Soumyabrata\smpl2bvh\CharacterAnimationTools")

from anim import amass
from anim import bvh
from anim.animation import Animation
from util import quat
import numpy as np

def save_as_npz(output_file, smpl_poses, smpl_trans, gender, fps):
    np.savez(output_file, trans=smpl_trans, gender=gender, mocap_framerate=fps, poses=smpl_poses)

# def reverse_animation_to_amass(anim: Animation, skel: Skel, scale: float = 100.0) -> dict:
def return_poses(output,quats,trans,fps,skel, scale: float = 100.0):
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
    # print("poses:",amass_dict['poses'])
    # save_as_npz(output, axangles, trans, gender, fps)

    return amass_dict["poses"]



# def save_as_npz(output_file, smpl_poses, smpl_trans, gender, fps):
#     np.savez(output_file, trans=smpl_trans, gender=gender, mocap_framerate=fps, poses=smpl_poses)

# def reverse_animation_to_amass(anim: Animation, skel: Skel, scale: float = 100.0) -> dict:
def return_trans(output,quats,trans,fps,skel, scale: float = 100.0):
    # quats = anim.quats
    # trans = anim.trans
    # fps = anim.fps

    # # Inverse of root quaternion transformation
    # trans_quat_inv_x = quat.inv(quat.from_angle_axis(-np.pi , [1, 0, 0]))
    # trans_quat_inv_y = quat.inv(quat.from_angle_axis(-np.pi / 2, [0, 1, 0]))
    # trans_quat_inv = quat.mul(trans_quat_inv_x, trans_quat_inv_y)  # Reverse order and apply inverse
    
    # flip_x_rotation = quat.from_angle_axis(-np.pi, [0, 0, 1])

    # trans_quat_inv_z = quat.inv(quat.from_angle_axis(0, [0, 0, 1]))
    # trans_quat_inv_y = quat.inv(quat.from_angle_axis(-np.pi/2 , [0, 1, 0]))
    # trans_quat_inv = quat.mul(flip_x_rotation,quat.mul(trans_quat_inv_y, trans_quat_inv_z))  # Combine inverses

    # Corrective rotation to make the humanoid upright
    correct_x_rotation = quat.from_angle_axis(np.pi / 2, [1, 0, 0])  
    correct_y_rotation = quat.from_angle_axis(-np.pi / 2, [0, 1, 0])  #(90,-90,0) --> faces -x axis ; (90,-90,180)--> faces +x axis
    correct_z_rotation = quat.from_angle_axis(np.pi , [0, 0, 1])  

    
    # Combine the corrective rotations
    trans_quat_inv = quat.mul(correct_z_rotation, quat.mul(correct_y_rotation, correct_x_rotation)) #comb1 #comb2->xyz #comb3->zxy #comb4->yxz
    # trans_quat_inv = quat.mul(correct_x_rotation, quat.mul(correct_y_rotation, correct_z_rotation)) #comb5->yzx #comb6->xzy
    
    
    root_rot_inv = quat.mul(trans_quat_inv[None], quats[:, 0])
    quats[:, 0] = root_rot_inv


    # # Inverse of translation transformation
    trans = trans @ quat.to_xform(quat.inv(trans_quat_inv)).T  #_1
    # trans = trans @ quat.to_xform(trans_quat_inv).T   #_2
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
    # print("trans:",amass_dict['trans'])
    # save_as_npz(output, axangles, trans, gender, fps)

    return amass_dict["trans"]


def process_directory(input_dir, output_dir, smpl_joint_names, bvh_to_smpl_map):
    # Iterate over each motion name directory
    for motion_name in os.listdir(input_dir):
        motion_path = os.path.join(input_dir, motion_name)
        if os.path.isdir(motion_path):
            # Iterate over each group name directory within the motion_name directory
            for group_name in os.listdir(motion_path):
                group_path = os.path.join(motion_path, group_name)
                if os.path.isdir(group_path):
                    # Iterate over each .bvh file within the group_name directory
                    for file in os.listdir(group_path):
                        if file.endswith('.bvh'):
                            bvh_file = os.path.join(group_path, file)
                            process_bvh_file(bvh_file, output_dir, smpl_joint_names, bvh_to_smpl_map, motion_name, group_name)

def process_bvh_file(bvh_file, output_dir, smpl_joint_names, bvh_to_smpl_map, motion_name, group_name, scale=100.0):
    anim_bvh = bvh.load(filepath=bvh_file, load_skel=True, load_pose=True)

    joints_to_use = []
    for joint_name in smpl_joint_names:
        for skel_joint in anim_bvh.skel.joints:
            if skel_joint.name in bvh_to_smpl_map and bvh_to_smpl_map[skel_joint.name] == joint_name:
                joints_to_use.append(anim_bvh.skel.joints.index(skel_joint))

    quats_subset = anim_bvh.quats[:, joints_to_use, :]

    # Construct the output path to match the input structure
    output_file = os.path.join(output_dir, motion_name, group_name, os.path.basename(bvh_file).replace('.bvh', '.npz'))
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    print(anim_bvh.fps)
    
    poses = return_poses(output=output_file,quats=quats_subset,trans=anim_bvh.trans,fps=anim_bvh.fps*30,skel=anim_bvh.skel)
    trans = return_trans(output=output_file,quats=quats_subset,trans=anim_bvh.trans,fps=anim_bvh.fps*30,skel=anim_bvh.skel)
    save_as_npz(output_file,poses,trans,"male",anim_bvh.fps*30)
    # reverse_animation_to_amass(output=output_file, quats=quats_subset, trans=anim_bvh.trans, fps=anim_bvh.fps*30, skel=anim_bvh.skel)


input_dir = r"C:\Users\Soumyabrata\Desktop\motions_PHC"
output_dir = r"C:\Users\Soumyabrata\Desktop\motions_PHC_npz"

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

process_directory(input_dir, output_dir, smpl_joint_names, bvh_to_smpl_map)
