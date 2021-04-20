
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
	string base = "/mnt/BigSlowBoi/DOCUMENTS/Projects/AutomaticBoresightCalibration/Debug_INPUT/";

	vector<Scene> scenes = LoadDebugData(base);

	cout << "Finished loading debug scenes\n";

	//Get the Shiftdown for distance measurement simplification (If doing geolation later, must revert this!)
	// This makes it relative to the first Orientation
	vector<double> shiftdown;
	find_apply_shiftdown(scenes, shiftdown);



	cout << "\n-------------------------Finished finding planes -------------------------------------------------------\n";
	cout << "Matching planes....\n";
	// TODO: MATCH PLANESgit
	UniquePlanes unique_planes = match_scenes(scenes);
	cout << "Done.\nRemoving infrequent planes....";

	//Remove less frequent planes.
	int num_removed = remove_unfrequent(unique_planes);
	cout << "Done.\n";

	// Create observation for estimations

	VectorXd x0 = VectorXd::Zero(6+unique_planes.unique_planes.size()*4);

	// Set the approximate values for the boresight parameters (given by NovAtel):
//	x0(0) = -1.661; // x
//	x0(1) = -1.272; // y
//	x0(2) = -2.4158; // z
//	x0(3) = 180; // omega
//	x0(4) = 0; // phi
//	x0(5) = 90; // kappa


	x0(0) = -0; // x
	x0(1) = -0; // y
	x0(2) = 0; // z
	x0(3) = 180; // omega
	x0(4) = 0; // phi
	x0(5) = 90; // kappa


	MatrixXd point_details = MatrixXd::Zero(1,1);
	MatrixXd scene_details = MatrixXd::Zero(1,1);
	MatrixXd plane_details = MatrixXd::Zero(1,1);
	create_bundle_observations(x0, scenes, unique_planes, point_details, scene_details, plane_details);


	// ------------------------------------------------------------------------------------------------------------------------------
	//									Least Squares Adjustment


	BoresightLS boresight_adjustment;

	MatrixXd temp_cl = MatrixXd::Ones(point_details.rows()+plane_details.rows(),1);


	//input the observations
	boresight_adjustment.setIter(40);
	boresight_adjustment.setTol(1.0e-5);
	boresight_adjustment.setAdjustmentDetails(point_details, plane_details, scene_details, x0);

	cout << "\n\nThe x0 is: " << boresight_adjustment.getx0() << endl;
	boresight_adjustment.setP(temp_cl);
//	boresight_adjustment.computeA();
//	cout << boresight_adjustment.A << endl;
	boresight_adjustment.iterate();
//	boresight_adjustment.computeLS();
//	boresight_adjustment.computeMisclosure();
//
//	cout << "\n\nThe W is: \n" << boresight_adjustment.getw() << endl;

//
//	cout << "\n\nThe d is: " << boresight_adjustment.getd() << endl;
	cout << "\n\nThe xh is:\n " << boresight_adjustment.getxh() << endl;
	cout << "\n\nThe mean w is: " << boresight_adjustment.getw().mean() << endl;
//	cout << "\n\nThe shiftdown is: \n";
//	print_vector(shiftdown);

	clog << "\n\n FINISHED CALIBRATION\n\n";


	ofstream myfile;
	myfile.open ("A_Full.txt");
	myfile << boresight_adjustment.A << endl;
	myfile.close();

	ofstream wfile;
	wfile.open ("w_Final.txt");
	wfile << boresight_adjustment.getw ()<< endl;
	wfile.close();


	cin.get();
	return(0);
}
