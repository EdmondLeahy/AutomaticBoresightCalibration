
#include "CalibrationFuntions.h"
//#include "BAFunctions.h"


//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{


	PointCloudXYZptr cloud_in_temp(new PointCloudXYZ);
	PointCloudXYZptr cloud_in(new PointCloudXYZ);
	PointCloudXYZptr cloud_out(new PointCloudXYZ);

	// ##########################################################

	PointCloudXYZptr plane1(new PointCloudXYZ);
	PointCloudXYZptr plane2(new PointCloudXYZ);
	PointCloudXYZptr plane3(new PointCloudXYZ);
	char * in_cloud_1 = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/O4_Planes/Cloud_Plane_0.pcd";
	char * in_cloud_2 = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/O4_Planes/Cloud_Plane_1.pcd";
	char * in_cloud_3 = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/O4_Planes/Cloud_Plane_2.pcd";

	Read_Lidar_points(in_cloud_1, plane1);
	Read_Lidar_points(in_cloud_2, plane2);
	Read_Lidar_points(in_cloud_3, plane3);

	*cloud_in_temp += *plane1;
	*cloud_in_temp += *plane2;
	*cloud_in_temp += *plane3;


	in_cloud_1 = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/O1_Planes/Cloud_Plane_0.pcd";
	in_cloud_2 = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/O1_Planes/Cloud_Plane_1.pcd";
	in_cloud_3 = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/O1_Planes/Cloud_Plane_2.pcd";

	Read_Lidar_points(in_cloud_1, plane1);
	Read_Lidar_points(in_cloud_2, plane2);
	Read_Lidar_points(in_cloud_3, plane3);

	*cloud_out += *plane1;
	*cloud_out += *plane2;
	*cloud_out += *plane3;


	// difference in EOP from Base to scene 4
	Matrix4d rel_orientation;
	double heading_diff = -112.8/RAD2DEG;
	double cos_theta = cos(heading_diff);
	double sin_theta = sin(heading_diff);
	rel_orientation << cos_theta, -sin_theta, 0, 0,
						sin_theta, cos_theta, 0, 0,
						0, 0, 1,0,
						0,0,0,1;
	transformPointCloud (*cloud_in_temp, *cloud_in, rel_orientation);

	print_matrix(rel_orientation);

	pcl::io::savePCDFileASCII("in_cloud_temp.pcd", *cloud_in_temp);
	pcl::io::savePCDFileASCII("in_cloud.pcd", *cloud_in);
	pcl::io::savePCDFileASCII("out_cloud.pcd", *cloud_out);


   // ################################################################



	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations (8000);
	icp.setTransformationEpsilon (1e-9);
//	icp.setMaxCorrespondenceDistance (0.05);
//	icp.setEuclideanFitnessEpsilon (1);
	icp.setRANSACOutlierRejectionThreshold (1.5);
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	// ############################
	PointCloudXYZptr rot_cloud_final(new PointCloudXYZ);
	transformPointCloud (*cloud_in, *rot_cloud_final, icp.getFinalTransformation());
	pcl::io::savePCDFileASCII("registered_cloud.pcd", Final);
	pcl::io::savePCDFileASCII("in_cloud_rotated.pcd", *rot_cloud_final);

	return (0);
}


//
//
//int main() {
//	/*
//	--------------------------- IF THE MAP FITS BORESIGHT CALIBRATION - MAIN  ------------------------------------------------------
//
//
//
//	--------------------------------------------------------------------------------------------------------------------------------
//
//	*/
//	// Program Header
//	fprintf(stdout, "\n----------------------IF THE MAP FITS BORESIGHT CALIBRATION----------------------------\n");
//	fprintf(stdout, "\n\tThe purpose of this program is to compute the Boresight calibration parameters of \n");
//	fprintf(stdout, "\ta system with GNSS/INS/LiDAR sensors \n\n\n");
//
//
//	fprintf(stdout, "    _________|T|_______         											\n");
//	fprintf(stdout, "   |,-----.,-----.,---.\        											\n");
//	fprintf(stdout, "   ||  IF || THE ||MAP \\       											\n");
//	fprintf(stdout, "   |`-----'|-----||-----\`----. 											\n");
//	fprintf(stdout, "   [       |    -||-   _|    (| 											\n");
//	fprintf(stdout, "   [  ,--. |_FITS||___/.--.   | 											\n");
//	fprintf(stdout, "   =-(( `))-----------(( `))-== 											\n");
//	fprintf(stdout, "      `--'             `--'     											\n");
//
//
//	fprintf(stdout, "\n---------------------------------------------------------------------------------------\n");
//
//
//	//DEBUG INPUT
//	string base = "/mnt/BigSlowBoi/DOCUMENTS/University/UniversityCourses/5thyear/ENGO500/Data/Data/Debug_INPUT/";
//
//	vector<Scene> scenes = LoadDebugData(base);
//
//	cout << "Finished loading debug scenes\n";
//
//	//Get the Shiftdown for distance measurement simplification
//	vector<double> shiftdown;
//	find_apply_shiftdown(scenes, shiftdown);
//
//	cout << "\n-------------------------Finished finding planes -------------------------------------------------------\n";
//	cout << "Matching planes....\n";
//	// TODO: MATCH PLANES
//	UniquePlanes unique_planes = match_scenes(scenes);
//	cout << "Done.\nRemoving infrequent planes....";
//	cout << "\n\nMapping vector PRE REMOVE:\n";
//	print_vector(unique_planes.mapping_vec);
//	//Remove less frequent planes.
//	int num_removed = remove_unfrequent(unique_planes);
//	cout << "\n\nDone. Removed " << num_removed << " infrequent planes.\n";
//
//	cout << "\n\nMapping vector POST REMOVE:\n";
//	print_vector(unique_planes.mapping_vec);
//
//
//	cout << "\n\n FINISHED CALIBRATION\n\n";
//
//	cin.get();
//	return(0);
//}
