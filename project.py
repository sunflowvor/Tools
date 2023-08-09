import os
import numpy as np
import cv2
import open3d as o3d
import math

def quaternion_to_rotation_matrix(q):  # x, y ,z ,w
    rot_matrix = np.array(
        [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
         [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
         [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]])
    return rot_matrix

color_map = {

    'road': [128, 64, 128],
    'bus': [0, 60, 100],
    'car': [0, 0, 142],
    'truck': [0, 0, 70],
    'sky': [70, 130, 180],
    'construction': [70, 70, 70],
    'vegetation': [107, 142, 35],
    'road_sign': [220, 220, 0],
    'fence': [190, 153, 153],
    'void': [0, 0, 0],
    'human': [220, 20, 60],

    'telegraph_pole': [153, 153, 153],
    'communication_pole': [250, 170, 30],
    'rut': [111, 74, 0],
    'soft_soil': [81, 0, 81],
    'gravel': [102, 102, 156],
    'road_edge': [244, 35, 232],
    'puddle': [250, 170, 160],
    'taft': [230, 150, 140],
    'mining_truck': [180, 165, 180],
    'wide_body_truck': [150, 100, 100],
    'excavator': [150, 120, 90],
    'pushdozer': [152, 251, 152],
    'watering_car': [255, 0, 0],
    'fuel_tank_car': [0, 0, 90],
    'electric_shovel': [0, 0, 110],
    'massif': [0, 80, 100],
    'blocking_stone': [0, 0, 230],
    'ice_cream_cone': [119, 11, 32],
    'cable': [0, 0, 142],

    'engineering_vehicle': [100, 20, 60],
    'road_testing_equipment': [255, 0, 0],
    'tussock': [0, 255, 0],
}

mask_img_path = "20230721_1382/mask/seg/OCT-00/scene_01"
lidar_path = "lidar"
k = np.array([1193.761331, 0.000000, 1019.402981, \
		0.000000, 1191.550281, 794.716538, \
		0.000000, 0.000000, 1.000000])
		
T = np.array([-0.00533616, -0.999986, 0.00064036, 0.504211,\
		0.0342852, -0.000822947, -0.999412, -0.118845,\
		0.999398, -0.00531107, 0.0342891, -0.105546,\
		0, 0, 0, 1])
		
lidar_path = "point"
sequence_lidar_path = "clouds"
lidar_lists = os.listdir(lidar_path)
lidar_lists = sorted(lidar_lists)

image_files = os.listdir(mask_img_path)
image_files = sorted(image_files)

odom_list = []
with open("odom.txt", "r") as odom_file:
	for line in odom_file.readlines():
		line = line.strip("\n").split(" ")
		pose_time = float(line[0])
		tx, ty, tz = line[1], line[2], line[3]
		qw, qx, qy, qz = line[4], line[5], line[6], line[7]
		odom_list.append([pose_time, float(tx), float(ty), float(tz),\
		    				float(qw), float(qx), float(qy), float(qz)])
		#assert 0 

pose_list = []
for i in range(len(lidar_lists)):
	list_file_path = os.path.join(lidar_path, lidar_lists[i])
	sequences_lidar_path = os.path.join(sequence_lidar_path, str(i))
	image_file = image_files[i]
	key_timestamp = image_file.split(".mask.png")[0]
	start_timestamp = float(key_timestamp) - 0.5
	#print(start_timestamp)
	#assert 0
	sequences_pose_list = [] 
	timestamp = start_timestamp
	for i in range(11):
		timestamp = timestamp + 0.1	  
		key_pose = None
		for time in range(len(odom_list) - 1):
			start_time = odom_list[time][0]
			end_time = odom_list[time + 1][0]
			if start_time < timestamp and end_time > timestamp and abs(start_time - timestamp) < abs(end_time - timestamp):
				key_pose = odom_list[time]
			elif start_time < timestamp and end_time > timestamp and abs(start_time - timestamp) > abs(end_time - timestamp):
				key_pose = odom_list[time + 1]
			if key_pose is not None:
				sequences_pose_list.append(key_pose)
				break
		if key_pose is None:
			print(timestamp)
			#assert 0 
	assert len(sequences_pose_list) == 11
	#assert 0 
	pose_list.append(sequences_pose_list)
#print(pose_list)

#concate multiple lidar frames

for i in range(len(lidar_lists)):
	list_file_path = os.path.join(lidar_path, lidar_lists[i])
	sequences_lidar_path = os.path.join(sequence_lidar_path, str(i))
	sequences_pose = pose_list[i]
	#print(os.listdir(sequences_lidar_path))
	#print(len(sequences_pose))
	assert len(os.listdir(sequences_lidar_path)) == (len(sequences_pose) - 1)

	key_point = o3d.io.read_point_cloud(list_file_path)

	clouds_list = []

	for j in range(11 - 1):
		if j == int(np.floor(11/2)):
			continue
		target_index = str(i).zfill(6) + "_" + str(j - 5) + ".pcd"
		target_point = o3d.io.read_point_cloud(os.path.join(sequence_lidar_path, str(i), target_index))
		key_pose = sequences_pose[int(np.floor(11/2))]
		target_pose = sequences_pose[j]
		#print(key_pose)
		#print(target_pose)
		#print(sequences_pose)
		#assert 0 
		#trans_tx = target_pose[1] - key_pose[1]
		#trans_ty = target_pose[2] - key_pose[2]
		#trans_tz = target_pose[3] - key_pose[3]

		quan_key = key_pose[4:]
		quan_target = target_pose[4:]
		rot_key = quaternion_to_rotation_matrix(quan_key)
		rot_target = quaternion_to_rotation_matrix(quan_target)

		trans_t_key = np.array([key_pose[1], key_pose[2], key_pose[3]])
		trans_t_target = np.array([target_pose[1], target_pose[2], target_pose[3]])
		print("key", trans_t_key)
		print("trget", trans_t_target)

		trans_t_key = np.matmul(rot_key.T, trans_t_key)
		trans_t_target = np.matmul(rot_key.T, trans_t_target)

		#trans_R = np.matmul(rot_target, rot_key.T)
		#trans_t_target = np.matmul(trans_R.T, trans_t_target)

		print("Akey", trans_t_key)
		print("Atrget", trans_t_target)

		trans_tx = trans_t_target[0] - trans_t_key[0]
		trans_ty = trans_t_target[1] - trans_t_key[1]
		trans_tz = trans_t_target[2] - trans_t_key[2]

		trans_R = np.matmul(rot_target, rot_key.T)
		trans_t = np.array([trans_tx, trans_ty, trans_tz])

		print(trans_t)

		#assert 0 
			
		target_point_np = np.asarray(target_point.points)
		#print(np.matmul(trans_R, target_point_np.transpose(1, 0)).shape)
		#assert 0 
		#target_point_np = np.matmul(trans_R, target_point_np.transpose(1, 0)) - np.array([2, 0, 0]).reshape(-1, 1)
		target_point_np = np.matmul(trans_R, target_point_np.transpose(1, 0)) + trans_t.reshape(-1, 1)
		target_point_np = target_point_np.transpose(1, 0)
		key_point_np = np.asarray(key_point.points)

		target_transed_pcd = o3d.geometry.PointCloud()
		target_transed_pcd.points = o3d.utility.Vector3dVector(target_point_np)

		clouds_list.append(target_transed_pcd)
		
		vis = o3d.visualization.Visualizer()
		vis.create_window()	
		vis.add_geometry(key_point)
		vis.add_geometry(target_transed_pcd)
		vis.run()

		assert 0 

	key_point = o3d.io.read_point_cloud(list_file_path)
	vis = o3d.visualization.Visualizer()
	vis.create_window()	
	vis.add_geometry(key_point)
	vis.run()
	

	assert 0 

assert 0 
#assert 0 

print(lidar_lists)
assert 0 


for i in range(len(image_files)):
	index = image_files[i].split(".mask.png")[0]
	img = cv2.imread(os.path.join(mask_img_path, image_files[i]))
	print(img.shape)
	lidar_files = os.listdir(os.path.join(lidar_path))
	for j in range(len(lidar_files)):
		lidar_index = lidar_files[j].split(".pcd")[0]
		if abs(float(lidar_index) - float(index)) < 0.01:
			lidar = o3d.io.read_point_cloud(os.path.join(lidar_path, lidar_files[j]))

			points = np.asarray(lidar.points)

			colors = np.zeros_like(points)

			for each_point_idx in range(points.shape[0]):
				each_point = points[each_point_idx]
				each_point = each_point.reshape(-1, 3)

				R = np.array([[-0.00533616, -0.999986, 0.00064036],[0.0342852, -0.000822947, -0.999412],[0.999398, -0.00531107, 0.0342891]]) 
					
				t = np.array([0.504211,-0.118845,-0.105546]).reshape(3,1)     


				P = np.hstack((R,t))


				alpha = 0.0
				beta = 0.0
				gama = 0.0

				Rx = np.array([[1, 0, 0],
						[0, math.cos(alpha), -math.sin(alpha)],
						[0, math.sin(alpha), math.cos(alpha)]])
				Ry = np.array([[math.cos(beta), 0, -math.sin(beta)],
						[0, 1, 0],
						[math.sin(beta), 0, math.cos(beta)]])
				Rz = np.array([[math.cos(gama), math.sin(gama), 0],
						[-math.sin(gama), math.cos(gama), 0],
						[0, 0, 1]])

				each_point = each_point[each_point[:, 0] > 0.0]
				each_point  = each_point[:,:3]

				#print(xyz.shape)
				#print(P.shape)
				each_point = each_point.transpose()
				uv = P.dot(np.vstack((each_point, np.ones((1, each_point.shape[1]), dtype=np.float32))))  # lidar R&T to cam
				xyzrt = np.dot(Rx, uv)
				xyzrt = np.dot(Ry, xyzrt)
				xyzrt = np.dot(Rz, xyzrt)

				#uv[:2] = uv[:2] / uv[2]
				xyzrt[:2] = xyzrt[:2] / xyzrt[2]

				fc = [1193.761331, 1191.550281]

				cc = [1019.402981, 794.716538]

				kc = [-0.096052, 0.086153, 0.000924, 0.000896, 0.000000]



				kk = np.array([[fc[0],0,cc[0]],[0,fc[1],cc[1]],[0,0,1]]) 
				# distortion
				#xn = uv[:2]
				xn = xyzrt[:2]
				rpow2 =xn[0]**2+xn[1]**2
				rd = 1+kc[0]*rpow2+kc[1]*rpow2**2+kc[4]*rpow2**3
				dx = np.array([2*kc[2]*xn[0]*xn[1]+kc[3]*(rpow2+2*xn[0]**2),kc[2]*(rpow2+2*xn[1]*xn[1])+2*kc[3]*xn[0]*xn[1]])
				xd = rd*xn + dx

				# project to image
				xp = np.dot(kk,np.vstack((xd,np.ones((1,xd.shape[1])))))
				uv = np.round(xp[:2])

				uv = np.int16(np.round(uv))
				#print("uv",uv)
				# keep the points inside image
				im_shape = img.shape
				keep = (uv[0, :] >= 0) & (uv[1, :] >= 0) & (uv[0, :] < im_shape[1]) & (uv[1, :] < im_shape[0])
				uv = uv.transpose()[keep]
				each_point = each_point.transpose()
				each_point = each_point[keep]

				# exchange [y,x] to [x,y]
				uv = uv[:,[1,0]]
				#im[uv[:,0],uv[:,1]] = 255

				#print(uv)
				if len(uv) != 0:

					colors[each_point_idx] = img[uv[0][0], uv[0][1], :] / 255
	colored_pcd = o3d.geometry.PointCloud()
	colored_pcd.points = o3d.utility.Vector3dVector(points)
	colored_pcd.colors = o3d.utility.Vector3dVector(colors)	
	vis = o3d.visualization.Visualizer()
	vis.create_window()	
	vis.add_geometry(colored_pcd)
	vis.run()
	o3d.io.write_point_cloud("save_pcd/" + index + ".pcd", colored_pcd)
	assert 0 
