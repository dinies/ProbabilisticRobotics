#generate a bunch of sample points

source "./multi_ICP_3d.m"
num_landmarks=100;
num_poses=5;
world_size=10;

# landmarks in a matrix, one per column
P_world=(rand(landmark_dim, num_landmarks)-0.5)*world_size;


# poses in an array of 4x4 homogeneous transform matrices
XR_true=zeros(4,4,num_poses);
XL_true=P_world;

# initialize 1st pose
XR_true(:,:,1)=eye(4);

# scaling coefficient for uniform random pose generation
# adjusts the translation to cover world_size
# adjusts the rotation to span the three angles;
rand_scale=eye(6);
rand_scale(1:3,1:3)*=(0.5*world_size);
rand_scale(4:6,4:6)*=pi;

for (pose_num=2:num_poses)
    xr=rand(6,1)-0.5;
    Xr=v2t(rand_scale*xr);
    XR_true(:,:,pose_num)=Xr;
endfor;

# generate an ideal number of measurement
# each pose observes each landmark
num_measurements=num_poses*num_landmarks;
Z=zeros(landmark_dim,num_measurements);
associations=zeros(2,num_measurements);

measurement_num=1;
for (pose_num=1:num_poses)
    Xr=XR_true(:,:,pose_num);
    R = Xr(1:3,1:3);
    t = Xr(1:3,4);
    for (landmark_num=1:num_landmarks)
	Xl=XL_true(:,landmark_num);
	associations(:,measurement_num)=[pose_num,landmark_num]';
	Z(:,measurement_num)= R*Xl + t;
	measurement_num++;
    endfor;
endfor


# apply a perturbation to each ideal pose (construct the estimation problem)
pert_deviation=1;
pert_scale=eye(6)*pert_deviation;
XR_guess=XR_true;
XL_guess=XL_true;

for (pose_num=2:num_poses)
    xr=rand(6,1)-0.5;
    dXr=v2t(pert_scale*xr);
    XR_guess(:,:,pose_num)=dXr*XR_guess(:,:,pose_num);
endfor;

#apply a perturbation to each landmark
dXl=(rand(landmark_dim, num_landmarks)-0.5)*pert_deviation;
XL_guess+=dXl;

