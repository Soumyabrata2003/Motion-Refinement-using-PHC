# Post-processing using Perpetual Humanoid Control for Real-time Simulated Avatars (on bvh files)

This is a modified version of PHC (Perpetual Humanoid Control for Real-time Simulated Avatars ICCV 23) including the conversion of mixamo format bvh files to AMASS-style npz files for motion refinement. It has been faithfully borrowed from the official coding implementation (https://github.com/ZhengyiLuo/PHC). We have primarily used _single_ _primitive_ for training and evaluating the samples since they lead to 100% success rate for imitation (as per PHC standards) and are easier to train than multiple primitives which is not automated in PHC. Our rationale for using a high quality motion imitator like PHC for refining sample motions can be attributed to its robustness to noisy data alongside its performance.


### Dependencies

To create the environment, follow the following instructions: 

1. Create new conda environment and install pytroch:


```
conda create -n isaac python=3.8
conda install pytorch torchvision torchaudio pytorch-cuda=11.6 -c pytorch -c nvidia
pip install -r requirement.txt
```

2. Download and setup [Isaac Gym](https://developer.nvidia.com/isaac-gym). 


3. [Optional if only inference] Download SMPL paramters from [SMPL](https://smpl.is.tue.mpg.de/). Put them in the `data/smpl` folder, unzip them into 'data/smpl' folder. Please download the v1.1.0 version, which contains the neutral humanoid. Rename the files `basicmodel_neutral_lbs_10_207_0_v1.1.0`, `basicmodel_m_lbs_10_207_0_v1.1.0.pkl`, `basicmodel_f_lbs_10_207_0_v1.1.0.pkl` to `SMPL_NEUTRAL.pkl`, `SMPL_MALE.pkl` and `SMPL_FEMALE.pkl`. Rename The file structure should look like this:

```

|-- data
    |-- smpl
        |-- SMPL_FEMALE.pkl
        |-- SMPL_NEUTRAL.pkl
        |-- SMPL_MALE.pkl

```


Make sure you have the SMPL paramters properly setup by running the following scripts:
```
python scripts/vis/vis_motion_mj.py
python scripts/joint_monkey_smpl.py
```

The SMPL model is used to adjust the height the humanoid robot to avoid penetnration with the ground during data loading. 

4. Use the following script to download trained models of original PHC project and sample data.

```
bash download_data.sh
```

this wil download amass_isaac_standing_upright_slim.pkl, which is a standing still pose for testing. 

To evaluate with your own SMPL data, see the script `scripts/data_process/convert_data_smpl.py`. Pay speical attention to make sure the coordinate system is the same as the one used  in simulaiton (with negative z as gravity direction). 


## Evaluation 



Evaluate on Bvh/AMASS:

```
## Shape + rotation + keypoint model
python phc/run_hydra.py learning=im_mcp exp_name=phc_shape_mcp_iccv epoch=-1 test=True env=env_im_getup_mcp robot=smpl_humanoid_shape robot.freeze_hand=True robot.box_body=False env.z_activation=relu env.motion_file=sample_data/amass_isaac_standing_upright_slim.pkl env.models=['output/HumanoidIm/phc_shape_pnn_iccv/Humanoid.pth'] env.num_envs=1  headless=False im_eval=True


## keypoint model
python phc/run_hydra.py learning=im_mcp exp_name=phc_kp_mcp_iccv epoch=-1 test=True env=env_im_getup_mcp robot=smpl_humanoid robot.freeze_hand=True robot.box_body=False env.z_activation=relu env.motion_file=sample_data/amass_isaac_standing_upright_slim.pkl env.models=['output/HumanoidIm/phc_kp_pnn_iccv/Humanoid.pth'] env.num_envs=1024 env.obs_v=7  im_eval=True
```

Evaluate single primitive model:

```
## Shape + rotation + keypoint model
python phc/run_hydra.py learning=im_pnn exp_name=phc_shape_pnn_iccv epoch=-1 test=True env=env_im_pnn robot=smpl_humanoid_shape robot.freeze_hand=True robot.box_body=False env.motion_file=sample_data/amass_isaac_standing_upright_slim.pkl  env.num_envs=1  headless=False


## keypoint model
python phc/run_hydra.py learning=im_pnn exp_name=phc_kp_pnn_iccv epoch=-1 test=True env=env_im_pnn env.motion_file=sample_data/amass_isaac_standing_upright_slim.pkl robot.freeze_hand=True robot.box_body=False env.num_envs=1 env.obs_v=7  headless=False
```


## Training



### Data Processing bvh

Clone [CAT](https://github.com/KosukeFukazawa/CharacterAnimationTools/tree/main) repo and convert mixamo-formatted bvh files to [AMASS](https://amass.is.tue.mpg.de/)-style npz format by running (note that this is for a single sample, however, it can be modified for directories easily) :

```
python scripts/data_process_bvh/bvh2smpl.py 
```

Run the following script on the unzipped data:


```
python scripts/data_process_bvh/process_sample_raw.py --dir [path_to_dataset] --out_dir [out_dir]
```

which dumps the data into the `bvh_db_smplh.pt` file. Then, run 

```
python scripts/data_process_bvh/process_sample_db.py
```

We further process these data into Motionlib format by running the following script:

```
python scripts/data_process_bvh/convert_sample_isaac.py
```


### Training PHC

#### Train single primitive

```
python phc/run_hydra_2.py learning=im_big exp_name=phc_prim env=env_im robot=smpl_humanoid env.motion_file=sample_data/amass_isaac_standing_upright_slim.pkl  
```


#### Train full PNN model 


Training PHC is not super automated yet, so it requires some (a lot of) manual steps, and invovles changing the config file a couple of times during training based on the training phase. The `phc_shape_pnn_train_iccv.yaml` config file provides a starting point for training primitives. 

First, we will train one primitive, and keep an eye on its performance (--has_eval) flag. In the config, the "training_prim" is the primitive that is being trained. This need to be updated accordingly. 

```
python phc/run_hydra.py learning=im_pnn_big env=env_im_pnn robot=smpl_humanoid env.motion_file=[motion_file] exp_name=[exp_name] 
```

After the performance plateaus, we will dump the most recent sequences that the primitives has failed on, and use them to train the next primitive. Here idx is the primitive that should be trained. 

```
python scripts/pmcp/forward_pmcp.py --exp [exp_name] --epoch [epoch] --idx {idx}
```
The above script will dump two files: one is the next hard sequences to learn, and anothere one is the checkpoint to resume with the copied primitive. 

To train the next primitive, run teh following script:
```
python phc/run_hydra.py learning=im_pnn_big env=env_im_pnn robot=smpl_humanoid env.motion_file=[motion_file] epoch=[include epoch+1 from previous step] env.fitting=True env.training_prim=1
```

Repeat this process until no hard sequences are left. Then, to train the fail-state recovery primitive on simple locomotion data. 


```
python phc/run_hydra.py learning=im_pnn_big exp_name=phc_shape_pnn_iccv env=env_im_pnn robot=smpl_humanoid env.motion_file=[motion_file] epoch=[include current epoch + 1 from the forward_pmcp step] env.fitting=True env.training_prim=[+1] env.zero_out_far=True env.zero_out_far_train=True env.getup_udpate_epoch={epoch}
```

After all primitives are trained, train the composer: 

```
python phc/run_hydra.py learning=im_mcp_big exp_name=[] env=env_im_getup_mcp robot=smpl_humanoid env.motion_file=[motion_file] env.models=['output/HumanoidIm/{exp_name}/Humanoid.pth']
```

When training the composer, you can repeat the process above (progressive mining hard sequences) to improve performance. 


You can also just train one model for imitation (no PNN):
```
python phc/run_hydra.py learning=im exp_name=phc_prim_iccv  env=env_im robot=smpl_humanoid_shape robot.freeze_hand=True robot.box_body=False env.motion_file=sample_data/amass_isaac_standing_upright_slim.pkl
```

## Trouble Shooting


### Multiprocessing Issues
See [this issue](https://github.com/ZhengyiLuo/PerpetualHumanoidControl/issues/17) for some discusssions. 

For the data loading part, try use: 

at [this line](https://github.com/ZhengyiLuo/PerpetualHumanoidControl/blob/8e01930fbcaa3efb9fb8b4752f2cf52f41dfe260/phc/utils/motion_lib_base.py#L235), bascially, uncomment: 

```
mp.set_sharing_strategy('file_system')
```

which should fix the issue. Though using file_system has caused me problems before as well.


### Success Rate
The success rate is reported as "eval_success_rate" in the wandb logging, not the "success_rate", which is a episodic success rate used during training. 


## Citation
If you find this work useful for your research, please cite:
```
@inproceedings{Luo2023PerpetualHC,
    author={Zhengyi Luo and Jinkun Cao and Alexander W. Winkler and Kris Kitani and Weipeng Xu},
    title={Perpetual Humanoid Control for Real-time Simulated Avatars},
    booktitle={International Conference on Computer Vision (ICCV)},
    year={2023}
}            
```

Also consider citing these prior works that are used in this project:

```
@inproceedings{rempeluo2023tracepace,
    author={Rempe, Davis and Luo, Zhengyi and Peng, Xue Bin and Yuan, Ye and Kitani, Kris and Kreis, Karsten and Fidler, Sanja and Litany, Or},
    title={Trace and Pace: Controllable Pedestrian Animation via Guided Trajectory Diffusion},
    booktitle={Conference on Computer Vision and Pattern Recognition (CVPR)},
    year={2023}
}     

@inproceedings{Luo2022EmbodiedSH,
  title={Embodied Scene-aware Human Pose Estimation},
  author={Zhengyi Luo and Shun Iwase and Ye Yuan and Kris Kitani},
  booktitle={Advances in Neural Information Processing Systems},
  year={2022}
}

@inproceedings{Luo2021DynamicsRegulatedKP,
  title={Dynamics-Regulated Kinematic Policy for Egocentric Pose Estimation},
  author={Zhengyi Luo and Ryo Hachiuma and Ye Yuan and Kris Kitani},
  booktitle={Advances in Neural Information Processing Systems},
  year={2021}
}

```

## References
This repository is built on top of the following amazing repositories:
* Main code framework is from: [IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)
* Part of the SMPL_robot code is from: [PHC](https://github.com/ZhengyiLuo/PHC) and [UHC](https://github.com/ZhengyiLuo/UniversalHumanoidControl)
* SMPL models and layer is from: [SMPL-X model](https://github.com/vchoutas/smplx)

Please follow the lisence of the above repositories for usage. 
