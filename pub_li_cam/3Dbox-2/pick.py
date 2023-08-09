import os
import shutil
import math

source_path = "./label"
source_files = os.listdir(source_path)

lidar_source = "../../../OCT-03/scene_08/lidar"
lidar_target = "./lidar"

image_source = "../../../OCT-03/scene_08/left_cam"
image_target = "./image"

for each_file in source_files:
	print(each_file)
	shutil.copy(os.path.join(lidar_source, each_file.replace("txt", "pcd")), os.path.join(lidar_target, each_file.replace("txt", "pcd")))
	
	index = each_file.split(".txt")[0]
	index_int = round(float(index))
	image_index = str(index_int) + ".000000.png"
	shutil.copy(os.path.join(image_source, image_index), os.path.join(image_target, image_index))
	
	
