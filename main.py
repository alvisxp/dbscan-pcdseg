import argparse
import numpy as np
import open3d as o3d
import sklearn.neighbors
# import dbscan_2 as dbscan

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
        newPcd.points = pcdPoints
        pcdArrayList.append(pcdPoints)

        if newPcdArray.size:
            x_max = max(newPcdArray[:, 0])
            x_min = min(newPcdArray[:, 0])
            y_max = max(newPcdArray[:, 1])
            y_min = min(newPcdArray[:, 1])
            z_max = max(newPcdArray[:, 2])
            z_min = min(newPcdArray[:, 2])
        
        cube = [
	        [x_min, y_min, z_min],
	        [x_max, y_min, z_min],
	        [x_min, y_max, z_min],
	        [x_max, y_max, z_min],
	        [x_min, y_min, z_max],
	        [x_max, y_min, z_max],
	        [x_min, y_max, z_max],
	        [x_max, y_max, z_max]]

        lines = [[0,1],[0,2],[1,3],[2,3],
	              [4,5],[4,6],[5,7],[6,7],
	              [0,4],[1,5],[2,6],[3,7]]     

        line_set = o3d.Lineset()

        line_set.points = o3d.Vector3dVector(cube)
        line_set.lines = o3d.Vector2iVector(lines)

        pcdArrayList.extend([line_set])
        
    o3d.draw_geometries(pcdArrayList)

if __name__ == "main":
    main()
