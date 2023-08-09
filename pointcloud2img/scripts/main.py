#-*- coding:utf-8 -*-

import cv2
import open3d as o3d
import numpy as np 
import math
from scipy import misc
import os

lookup_table = np.array([[51,51,255],[51,102,255],[51,153,255],[51,204,255],[51,255,255],[51,255,204],
                        [51,255,153],[51,255,102],[51,255,51],[102,255,51],[153,255,51],[204,255,51],
                        [255,255,51],[255,204,51],[255,153,51],[255,102,51],[255,51,51],[255,51,102],
                        [255,51,153],[255,51,204],[255,51,255],[204,51,255],[102,51,255],[255,255,255]])

def main():
    pcd_path = "./lidar"
    pcd_files = os.listdir(pcd_path)
    pcd_files.sort(key=lambda x:float(x[:-4]))

    img_path = "./left_cam"
    img_files = os.listdir(img_path)
    img_files.sort(key=lambda x:float(x[:-4]))

    i = 0 
    for index in pcd_files:

        file_path = os.path.join("./lidar", index)
	    #file_path = '/Users/songxiaokang/phd_project/cloudpoint2img/000010.pcd'
        pcd = o3d.io.read_point_cloud(file_path)
        xyz = np.asarray(pcd.points)

        #o3d.visualization.draw_geometries([pcd])

        image=cv2.imread(os.path.join("./left_cam", img_files[i]))
        #image=cv2.imread('/Users/songxiaokang/phd_project/cloudpoint2img/000010.png')
        print(image.shape)

        R = np.array([[-0.00533616,-0.999986,0.00064036],[0.0342852,-0.000822947,-0.999412],[0.999398,-0.00531107,0.0342891]]) 
                
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

        xyz = xyz[xyz[:, 0] > 0.0]
        xyz  = xyz[:,:3]

        print(xyz.shape)
        print(P.shape)
        xyz = xyz.transpose()
        uv = P.dot(np.vstack((xyz, np.ones((1, xyz.shape[1]), dtype=np.float32))))  # lidar R&T to cam
        xyzrt = np.dot(Rx, uv)
        xyzrt = np.dot(Ry, xyzrt)
        xyzrt = np.dot(Rz, xyzrt)

        #uv[:2] = uv[:2] / uv[2]
        xyzrt[:2] = xyzrt[:2] / xyzrt[2]

        #baoli
        #2484.043102 0.000000 1023.870456 0.000000
        #0.000000 2484.378792 776.672429 0.000000
        #0.000000 0.000000 1.000000 0.000000
        #kc = [-0.100799, 0.152743, -0.001177, -0.000237, 0.000000]


        #kitti
        #7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 4.485728000000e+01 
        #0.000000000000e+00 7.215377000000e+02 1.728540000000e+02 2.163791000000e-01 
        #0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 2.745884000000e-03


        fc = [1207.674933, 1206.969351]

        cc = [1018.443112, 791.511478]

        kc = [-0.095861, 0.082828, -0.000168, 0.000087, 0.000000]



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
        im_shape = image.shape
        keep = (uv[0, :] >= 0) & (uv[1, :] >= 0) & (uv[0, :] < im_shape[1]) & (uv[1, :] < im_shape[0])
        uv = uv.transpose()[keep]
        xyz = xyz.transpose()
        xyz = xyz[keep]

        # exchange [y,x] to [x,y]
        uv = uv[:,[1,0]]
        #im[uv[:,0],uv[:,1]] = 255


        depth = np.uint8(xyz[:,0]*(-1))/11
        depth = np.uint(depth)
        print(uv)
        print(uv[:, 0])
        red = [0, 255, 0]
        print(image.shape)

        #image[uv[:,0],uv[:,1]] = lookup_table[depth[:]]
        image[uv[:,0],uv[:,1]] = red

        image[uv[:,0] - 1,uv[:,1]] = red
        #image[uv[:,0] + 1,uv[:,1]] = red
        image[uv[:,0], uv[:,1] - 1] = red
        #image[uv[:,0], uv[:,1] + 1] = red
        #image[uv[:,0] - 1,uv[:,1] + 1] = red
        #image[uv[:,0] + 1,uv[:,1] - 1] = red
        #image[uv[:,0] + 1, uv[:,1] + 1] = red
        image[uv[:,0] - 1, uv[:,1] - 1] = red
        #imfuse[uv[:,0],uv[:,1]] = [0,0,255]

        save_image_name = "./test/" + str(i).zfill(6) + ".png"
        cv2.imwrite(save_image_name, np.uint8(image), [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        #cv2.imwrite('testsplit' + im_name, np.uint8(im), [int(cv2.IMWRITE_PNG_COMPRESSION), 0])

        i = i + 1

    assert 0 

"""
import numpy as np
import cv2
import math
import os

def mkdir(path):
    folder = os.path.exists(path)

    if not folder:
        os.mkdir(path)
    else: 
        print ('the floder is exist')

def project_to_image(xyz,P,kk,kc,Rx,Ry,Rz):
    # project to camera coordinates
    uv = P.dot(np.vstack((xyz, np.ones((1, xyz.shape[1]), dtype=np.float32))))  # lidar R&T to cam
    xyzrt = np.dot(Rx, uv)
    xyzrt = np.dot(Ry, xyzrt)
    xyzrt = np.dot(Rz, xyzrt)
    #uv[:2] = uv[:2] / uv[2]
    xyzrt[:2] = xyzrt[:2] / xyzrt[2]

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

    return uv

if __name__ == '__main__':


    lookup_table = np.array([[51,51,255],[51,102,255],[51,153,255],[51,204,255],[51,255,255],[51,255,204],
                            [51,255,153],[51,255,102],[51,255,51],[102,255,51],[153,255,51],[204,255,51],
                            [255,255,51],[255,204,51],[255,153,51],[255,102,51],[255,51,51],[255,51,102],
                            [255,51,153],[255,51,204],[255,51,255],[204,51,255],[102,51,255],[255,255,255]])
        
    # intrinsic
   
    #fc = [3.144151860802466e+02, 3.161926681666730e+02] # focal length
    fc = [3.06605e+02, 3.06605e+02]
    #fc = [2.00000e+02, 2.00000e+02]
    #cc = [0.640453580151516e+03, 3.600258620867757e+02] # camera center 
    cc = [1250, 425]
    #kc = [0.007886188733239, -7.547732266564628e-04, 0.0, 0.0, 0.0] # distortion coefficient
    kc = [0.0, 0.0, 0.0, 0.0, 0.0]
    

    kk = np.array([[fc[0],0,cc[0]],[0,fc[1],cc[1]],[0,0,1]])      

    # t = np.array([0.004697 , -1.071230 , -2.153231]).reshape(3, 1)
    # R = np.array([ [ 0.252074 , -0.546849 , 0.798383],
    #                [-0.094748 , -0.835007 , -0.542020 ],
    #                [0.963058 , 0.060984 , -0.262297] ])

    t = np.array([0.045524, -0.287143, -2.194141]).reshape(3,1)
    R = np.array([[0.041192, 0.999129, 0.006702],
                  [0.362500, -0.008694, -0.931943],
                  [-0.931073, 0.040819, -0.362542]])
    
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

    img_path = "./data/image_dis/"
    save_path = "./output/"
    lidar_root = "./data/txt/"
    img_suffix = ".jpg"
    img_prefix = "image"

    mkdir(save_path)

    for img_name_tmp in os.listdir(img_path):
        img_name_split = img_name_tmp.split(img_suffix)
        if(len(img_name_split) <= 1):
            continue
        
        im_name = img_path + img_name_tmp
        # im_shape = cv2.imread('un_image' + str(num) + '.jpg').shape #720L, 1280L, 3L
        # imfuse = cv2.imread('un_image' + str(num) + '.jpg')

        im_shape = cv2.imread(im_name).shape #720L, 1280L, 3L
        imfuse = cv2.imread(im_name)
        imfuse = np.uint8(imfuse)

        im = np.zeros(( im_shape[0],im_shape[1],im_shape[2]))
        num = img_name_split[0].split(img_prefix)[-1]
        num_int = int(num)
        lidar_path = lidar_root  + str(num_int).zfill(6) + '.txt'
        print("####",lidar_path)
        #print("xyz",xyz.shape)
        xyz = np.loadtxt(lidar_path)
        print("xyz",xyz.shape)

        # remove all points behind image plane (approximation)
        xyz = xyz[xyz[:, 0] <= 0.0]
        
        xyz  = xyz[:,:3]
        
        # xyz = xyz[xyz[:, 1] <= -1.0]
        uv = np.int16(np.round(project_to_image(xyz.transpose(),P,kk,kc,Rx,Ry,Rz)))
        print("uv",uv.shape)
        # keep the points inside image
        keep = (uv[0, :] >= 0) & (uv[1, :] >= 0) & (uv[0, :] < im_shape[1]) & (uv[1, :] < im_shape[0])
        uv = uv.transpose()[keep]
        xyz = xyz[keep]
        
        # exchange [y,x] to [x,y]
        uv = uv[:,[1,0]]
        im[uv[:,0],uv[:,1]] = 255
        
        
        depth = np.uint8(xyz[:,0]*(-1))/5
        depth = np.uint(depth)
        

        imfuse[uv[:,0],uv[:,1]] = lookup_table[depth[:]]
        #imfuse[uv[:,0],uv[:,1]] = [0,0,255]
        im[uv[:,0],uv[:,1]] = lookup_table[depth[:]]
        #im[uv[:,0],uv[:,1]] = [0,0,255]
        save_image_name = save_path + img_name_tmp 
        cv2.imwrite(save_image_name, np.uint8(imfuse), [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        #cv2.imwrite('testsplit' + im_name, np.uint8(im), [int(cv2.IMWRITE_PNG_COMPRESSION), 0])

"""

"""

if __name__ == "__main__":
    main()
def main():
    pcd_load = o3d.io.read_point_cloud('./1660379436.798767872.pcd')
    xyz_load = np.asarray(pcd_load.points)

    image=cv2.imread('./1660379436.900000.png')

    R=np.array([[-0.0105336,-0.999361,-0.0341672],[0.0373452,0.033752,-0.998732],[0.999247,-0.0117962, 0.0369658]])
    t=np.array([[0.51842],[-0.186819],[-0.13145]])
    camera=np.array(([1207.674933,0,1018.443112],[0,1206.969351,791.511478],[0,0,1]),dtype=np.double)

    pcd_image = np.dot(xyz_load,R) + t



if __name__ == "__main__":
    main()
"""

if __name__ == "__main__":
    main()


