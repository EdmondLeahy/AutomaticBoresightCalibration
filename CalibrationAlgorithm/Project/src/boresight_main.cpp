
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

	//Get the Shiftdown for distance measurement simplification
	vector<double> shiftdown;
	find_apply_shiftdown(scenes, shiftdown);

	clog << "\n-------------------------Finished finding planes -------------------------------------------------------\n";
	clog << "Matching planes....\n";
	// TODO: MATCH PLANES
	UniquePlanes unique_planes = match_scenes(scenes);
	clog << "Done.\nRemoving infrequent planes....";
	clog << "\n\nMapping vector PRE REMOVE:\n";
	print_vector(unique_planes.mapping_vec);
	//Remove less frequent planes.
	int num_removed = remove_unfrequent(unique_planes);
	clog << "\n\nDone. Removed " << num_removed << " infrequent planes.\n";

	clog << "\n\nMapping vector POST REMOVE:\n";
	print_vector(unique_planes.mapping_vec);


	clog << "\n\n FINISHED CALIBRATION\n\n";

	cin.get();
	return(0);
}
