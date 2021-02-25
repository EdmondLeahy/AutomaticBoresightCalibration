
#include "CalibrationFuntions.h"
//#include "BAFunctions.h"




int main() {
	/*
	--------------------------- IF THE MAP FITS BORESIGHT CALIBRATION - MAIN  ------------------------------------------------------



	--------------------------------------------------------------------------------------------------------------------------------

	*/
	// Program Header
	fprintf(stdout, "\n----------------------IF THE MAP FITS BORESIGHT CALIBRATION----------------------------\n");
	fprintf(stdout, "\n\tThe purpose of this program is to compute the Boresight calibration parameters of \n");
	fprintf(stdout, "\ta system with GNSS/INS/LiDAR sensors \n\n\n");


	fprintf(stdout, "    _________|T|_______         											\n");
	fprintf(stdout, "   |,-----.,-----.,---.\        											\n");
	fprintf(stdout, "   ||  IF || THE ||MAP \\       											\n");
	fprintf(stdout, "   |`-----'|-----||-----\`----. 											\n");
	fprintf(stdout, "   [       |    -||-   _|    (| 											\n");
	fprintf(stdout, "   [  ,--. |_FITS||___/.--.   | 											\n");
	fprintf(stdout, "   =-(( `))-----------(( `))-== 											\n");
	fprintf(stdout, "      `--'             `--'     											\n");


	fprintf(stdout, "\n---------------------------------------------------------------------------------------\n");


	//DEBUG INPUT
	string base = "/mnt/BigSlowBoi/DOCUMENTS/University/UniversityCourses/5thyear/ENGO500/Data/Data/Debug_INPUT/";

	vector<Scene> scenes = LoadDebugData(base);

	cout << "Finished loading debug scenes\n";
//
	//	PointCloudXYZptr combined_cloud_sc1(new PointCloudXYZ) = scenes[0].planes[0].points_on_plane;
	PointCloudXYZptr cloud_s1(new PointCloudXYZ), cloud_s2(new PointCloudXYZ);
	*cloud_s1 += *scenes[0].planes[0].points_on_plane;
	*cloud_s1 += *scenes[0].planes[1].points_on_plane;
	*cloud_s1 += *scenes[0].planes[2].points_on_plane;

	*cloud_s2 += *scenes[1].planes[0].points_on_plane;
	*cloud_s2 += *scenes[1].planes[1].points_on_plane;
	*cloud_s2 += *scenes[1].planes[2].points_on_plane;
//

	cout << "size of cloud 1: " << cloud_s1->points.size() << endl;
	cout << "size of cloud 2: " << cloud_s2->points.size() << endl;
//
////	print_cloud(printable_cloud);
//
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_s1);
	icp.setInputTarget(cloud_s2);
//
	PointCloudXYZ Final;
	icp.align(Final);
//
//	cout << "Converged: " << icp.hasConverged() << endl;
//	cout << "Score: " << icp.getFitnessScore() << endl;
//	cout << "Result: \n" << icp.getFinalTransformation() << endl;
//
//	Eigen::Matrix4d icp_transf = Eigen::Matrix4d::Identity ();
//	icp_transf = icp.getFinalTransformation ().cast<double>();
//
//	printf ("Rotation matrix :\n");
//	printf ("    | %6.3f %6.3f %6.3f | \n", icp_transf (0, 0), icp_transf (0, 1), icp_transf (0, 2));
//	printf ("R = | %6.3f %6.3f %6.3f | \n", icp_transf (1, 0), icp_transf (1, 1), icp_transf (1, 2));
//	printf ("    | %6.3f %6.3f %6.3f | \n", icp_transf (2, 0), icp_transf (2, 1), icp_transf (2, 2));
//	printf ("Translation vector :\n");
//	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", icp_transf (0, 3), icp_transf (1, 3), icp_transf (2, 3));
//
//
//
//	double diff_x = scenes[0].scene_orientation.X - scenes[1].scene_orientation.X;
//	double diff_y = scenes[0].scene_orientation.Y - scenes[1].scene_orientation.Y;
//	double diff_z = scenes[0].scene_orientation.Z - scenes[1].scene_orientation.Z;
//	printf ("Scene Orientation Translation :\n");
//	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", diff_x, diff_y, diff_z);
//
//	double d_ins = dist(diff_x, diff_y, diff_z);
//	double d_reg = dist(icp_transf (0, 3), icp_transf (1, 3), icp_transf (2, 3));
//
//	cout << "Dist INS:\t" << d_ins << endl;
//	cout << "Dist REG:\t" << d_reg << endl;

	clog << "\n\n FINISHED CALIBRATION\n\n";

	cin.get();
	return(0);
}
