
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

	//Get the Shiftdown for distance measurement simplification (If doing geolation later, must revert this!)
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

//	cout << "\n\nMapping vector POST REMOVE:\n";
	print_vector(unique_planes.mapping_vec);

	// Create observation for estimations
	MatrixXd point_details, scene_details, plane_details;
	create_bundle_observations(scenes, unique_planes, point_details, scene_details, plane_details);

	// ------------------------------------------------------------------------------------------------------------------------------
	//									Least Squares

//	BoresightLS boresight_adjustment;

	//input the observations
//	boresight_adjustment.setAdjustmentDetails(point_details, scene_details, plane_details);


	clog << "\n\n FINISHED CALIBRATION\n\n";

	cin.get();
	return(0);
}
