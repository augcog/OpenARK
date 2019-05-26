from __future__ import print_function
import os, sys
import numpy as np

print ("OpenARK SMPL Model Converter Utility v0.2, created by Alex Yu 2018-19")
print ("This utility converts SMPL pickled model files (.pkl) to PCL point cloud files (.pcd) + an easy-to-parse skeleton information file.\n")
if len(sys.argv) < 2:
    print ("Usage: python extract.py pkl_file_name [dest_dir=pwd]")
    sys.exit(0)
    
SRC_PATH = sys.argv[1]
if len(sys.argv) >= 3:
    DEST_DIR = sys.argv[2]
    if not os.path.exists(DEST_DIR):
        os.makedirs(DEST_DIR)
else:
    DEST_DIR = ""

MODEL_FILE = os.path.join(DEST_DIR, 'model.pcd')
SKEL_FILE = os.path.join(DEST_DIR, 'skeleton.txt')
JREG_FILE = os.path.join(DEST_DIR, 'joint_regressor.txt')
SHAPEKEY_DIR = os.path.join(DEST_DIR, 'shapekey')

if not os.path.exists(SHAPEKEY_DIR):
    os.makedirs(SHAPEKEY_DIR)
    
def write_pcd(path, data):
    f = open(path, 'w')
    f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
    f.write("WIDTH " + str(data.shape[0]) +"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS " + str(data.shape[0]) +"\nDATA ascii\n")
    np.savetxt(f, data, fmt="%.18f")
    f.close()
    
def write_skel(path, joints, data):
    f = open(path, 'w')
    
    f.write("%s %s\n" % (joints.shape[0], data.shape[0]))
    
    #names = ["PELVIS", "L_HIP", "L_KNEE", "L_ANKLE", "L_FOOT", "R_HIP", "R_KNEE", "R_ANKLE", "R_FOOT", "SPINE1", "SPINE2", "SPINE3", "NECK", "HEAD", "L_COLLAR", "L_SHOULDER", "L_ELBOW", "L_WRIST", "L_HAND", "R_COLLAR", "R_SHOULDER", "R_ELBOW", "R_WRIST", "R_HAND"]
    #parents = [-1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 11, 14, 15, 16, 17, 11, 19, 20, 21, 22]
    names = ["PELVIS", "R_HIP", "L_HIP", "SPINE1", "R_KNEE", "L_KNEE", "SPINE2", "R_ANKLE", "L_ANKLE", "SPINE3", "R_FOOT", "L_FOOT", "NECK", "R_COLLAR", "L_COLLAR", "HEAD", "R_SHOULDER", "L_SHOULDER", "R_ELBOW", "L_ELBOW", "R_WRIST", "L_WRIST", "R_HAND", "L_HAND"]
    parents = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]
    for i in range(joints.shape[0]):
        f.write("%s %s %s %.18f %.18f %.18f\n" % (i, parents[i], names[i], joints[i][0], joints[i][1], joints[i][2]))

    for row in data:
        nonzero = []
        for i in range(row.shape[0]):
            if row[i] > 0:
                nonzero.append(i)
        f.write("%s" % len(nonzero));
        for i in nonzero:
             f.write(" %s %.18f" % (i, row[i]))
        f.write("\n")
    
    f.close()
    
def write_jreg(path, data):
    f = open(path, 'w')
    f.write('%s\n' % data.shape[0])
    import scipy
    data_rows = scipy.sparse.csr_matrix(data)
    for i in range(data_rows.shape[0]):
        row = data_rows[i]
        f.write(str(row.nnz))
        for j in range(row.nnz):
            f.write(' %s %.18f' % (row.indices[j], row.data[j]))
        f.write('\n')
    
import pickle

f = open(SRC_PATH, 'rb')
try:
    d = pickle.load(f, encoding='latin1')
except:
    d = pickle.load(f)

# {'f': (13776, 3), 'kintree_table': (2, 24), 'J': (24, 3), 'weights_prior': (6890, 24), 'weights': (6890, 24), 'posedirs': (6890, 3, 207), 'shapedirs': (6890, 3, 10), 'v_template': (6890, 3)}

print("Writing model template...")
write_pcd(MODEL_FILE, d['v_template'])

print("Writing skeleton...")
write_skel(SKEL_FILE, d['J'], d['weights'])

print("Writing joint regressor...")
write_jreg(JREG_FILE, d['J_regressor'])

print("Writing shape shapekeys...")

shapedirs = d['shapedirs']
for i in range(shapedirs.shape[2]):
    path = os.path.join(SHAPEKEY_DIR, "shape%03d.pcd" % i)
    write_pcd(path, shapedirs[:,:,i])
    if i % 5 == 4:
        print("%s of %s shape shapekeys written" % (i + 1, shapedirs.shape[2]))

# not required for OpenARK, but if you need you can uncomment the section below    
# print("Writing pose shapekeys...")
    
# posedirs = d['posedirs']
# for i in range(posedirs.shape[2]):
    # path = os.path.join(SHAPEKEY_DIR, "pose%03d.pcd" % i)
    # write_pcd(path, posedirs[:,:,i])
    # if i % 20 == 19:
        # print("%s of %s pose shapekeys written" % (i + 1, posedirs.shape[2]))
    
print("All done.")
