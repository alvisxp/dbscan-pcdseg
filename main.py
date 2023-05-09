import argparse
import numpy as np
import open3d as o3d
import sklearn.neighbors
import dbscan_2 as dbscan

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", dest="filename")
    parser.add_argument("--eps", dest="eps")
    parser.add_arguemnt("--minPts", dest="minPts")
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.filename)

    downsampRatio = np.array(pcd.points).shape[0]/ 10000
    
    pcd = o3d.geometry.unifrom_down_sample(pcd, every_k_points= int(downsampRatio))
    pcdArray = np.asarray(pcd.points)

    above_ground = [], ground_list = []

    for point in pcdArray:
        if point[2] > 1.5:
            above_ground.append(point)
        else:
            ground_list.append(point)
    ground = np.asarray(ground_list)
    above_ground = np.asarray(above_ground)

    labels = dbscan.DBSCAN(above_ground, args.eps, args.minPts)
    labels = np.array(labels)

    pcdArrayList = []

    for label in range(len(set(labels))):
        newPcdArray = above_ground[np.where(labels == label)[0]]
        pcdPoints = o3d.utility.Vector3dVector(newPcdArray)
        newPcd = o3d.PointCloud()
