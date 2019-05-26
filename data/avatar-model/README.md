# Avatar Module - SMPL Human Body Model Instructions
Due to licensing restrictions, we are unable to directly provide the SMPL human model files required by the avatar module here. Instead, we ask the user to download the original files from the SMPL website and then run the included Python script for extracting the models to a format usable by OpenARK. Please follow the instructions below:

- Register and download one of:
    - Gender-neutral model from <smplify.is.tue.mpg.de/pre_download>. Download *SMPLify_Code_V2.zip*. *Recommended*
    - Male/Female model from <http://smpl.is.tue.mpg.de/downloads>. Use the *SMPL For Python Users* version.
    
    You must accept the respective licenses for these models on the websites before doing so. Be warned that the license are non-commercial and quite restrictive.
    
- Copy the desired model's pickle file: `smplify_public/code/models/basicModel_neutral_lbs_10_207_0_v1.0.0.pkl` OR `smpl/models/basicModel_x_lbs_10_207_0_v1.0.0.pkl` to the directory of this README file.

- If not already present, install Python 3 and the libraries `numpy`, `scipy`, and `chumpy`.

- Open a terminal and `cd` to this directory.

- Enter `python3 extract.py smpl_model_name_here.pkl` and wait for the model to extract. Python 3 is recommended, although Python 2 may work.

## References

[1] Federica Bogo, Angjoo Kanazawa, Christoph Lassner, Peter Gehler, Javier
Romero, and Michael J. Black. Keep it SMPL: Automatic estimation of 3D
human pose and shape from a single image. In Computer Vision – ECCV 2016, Lecture Notes in Computer Science. Springer International Publishing,
October 2016.

[2] Matthew Loper, Naureen Mahmood, Javier Romero, Gerard Pons-Moll, and
Michael J. Black. SMPL: A skinned multi-person linear model.
ACM Trans. Graphics (Proc. SIGGRAPH Asia), 34(6):248:1–248:16, October 2015.

## Extracted Model Structure

**basicModel_neutral_lbs_10_207_0_v1.0.0/\*** Gender neutral model (recommended)

**basicModel_m_lbs_10_207_0_v1.0.0/\*** Male model

**basicModel_f_lbs_10_207_0_v1.0.0/\*** Female model

-------

**basicModel_x_lbs_10_207_0_v1.0.0/model.pcd** 

PCL point cloud file containing list of points in model

**basicModel_x_lbs_10_207_0_v1.0.0/skeleton.txt**  

Skeleton data. *Skeleton file spec:*  

- Line 1 contains two integers: the number of joints (M) and the total number of points in the model (N).

- Each line in the next section from line 2 to line 1+M consists of (joint ID, parent joint ID, joint name, and XYZ position (m)).

- The final section from line 1+M+1 to line 1+M+N contains blend skinning weights for each point in the order they appear in the PCD. Each line contains (number of entries, [joint ID, weight, joint ID, weight...]) where weights should sum to 1.

**basicModel_x_lbs_10_207_0_v1.0.0/shapekey/\*** 

Contains shape keys, which store PCA info about body shape/pose.  They are stored as point clouds, and we should linearly interpolate between these points and the default model points (with matching indices) to modify shape.

------
**skeleton.txt** 

Contains generic info about skeleton structure (ID, parent, name of joints)