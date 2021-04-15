#ifndef LS_H
#define LS_H
#endif

#include "LeastSquares.h"
//#include "CalibrationFunctions.h"

#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Dense> //for Eigen library
#include <Eigen/Core>
#include <fstream> //for read/write files
#include <algorithm>
#include <stdexcept>
#include <string>
#include <iomanip> 

#define MaxMatSize 10000000
#define PI 3.14159265358979323846 
#define sind(x) (sin(fmod((x),360) * PI / 180))
#define cosd(x) (cos(fmod((x),360) * PI / 180))


using namespace std;
using namespace Eigen;


class BoresightLS : public LeastSquares
{
public:
	const double LIGHTSPEED = (299792458.0);
	BoresightLS();
	void setAdjustmentDetails(MatrixXd point_details_in, MatrixXd plane_details_in, MatrixXd scene_details_in, VectorXd x0_est);

	void computeA();
	void computew();
	void computefx();


private:
	int numPlanes, numLidPts, numScenes;
	MatrixXd plane_details, scene_details, point_details, GNSS_INS_data;
	RowVectorXd compute_angle_Arow(int j, int i, int k);
	RowVectorXd compute_dist_Arow(int i, int j);//Function to compute A
	void computeAandw();
	//Function to compute a row of A for a plane
	MatrixXd computeAPlane(int u, int planeNum, double n_xpg, double n_ypg, double n_zpg);
	//Function to compute a row of A for a lidar point
	MatrixXd computeAPt(int pt_index);
	//Function to compute elements of a rotation matrix
	MatrixXd RotMatElements(double w, double phi, double K);
	//Function to compute the derivatives of a point equation wrt rotation matrix elements for Rbjg
	MatrixXd PtEqnWrtRotbjg(double x_Sjb, double y_Sjb, double z_Sjb, double w_Sb, double phi_Sb, double K_Sb, double x_sj, double y_sj, double z_sj, double n_xpg, double n_ypg, double n_zpg);
	//Function to compute the derivatives of a point equation wrt rotation matrix elements for RSb
	MatrixXd PtEqnWrtRotSb(double w_bjg, double phi_bjg, double K_bjg, double x_sj, double y_sj, double z_sj, double n_xpg, double n_ypg, double n_zpg);
	//Function to compute a row of w for a lidar point
	MatrixXd computewPt(double x_Sjb, double y_Sjb, double z_Sjb,
		double w_Sb, double phi_Sb, double K_Sb,
		double n_xpg, double n_ypg, double n_zpg, double d_p,
		double x_bjg, double y_bjg, double z_bjg,
		double w_bjg, double phi_bjg, double K_bjg,
		double x_sj, double y_sj, double z_sj);
	//Function to compute a row of w for a plane
	MatrixXd computewPlane(double n_xpg, double n_ypg, double n_zpg);

	//Function to compute derivatives of rotation matrix elements wrt rotation angles
	MatrixXd RotWrtAngles(double w, double phi, double K);





};


void RotationMatrix(double Omega, double Phi, double Kappa, MatrixXd & Rot_g2i); //Takes 3 Anges, makes 3x3 matrix

//Function to fill a matrix with info from vector of vectors
void vec2mat(vector<RowVectorXd>& vec, MatrixXd& mat, int cols);


VectorXd bring_to_ground(VectorXd eop, VectorXd x0, VectorXd n);
VectorXd bring_plane_to_ground(VectorXd eop, VectorXd x0, VectorXd n);


double get_plane_d(VectorXd n_plane);

//Function to compute 6 rows of A for a scan
MatrixXd computeAScan(int u, int numPlanes, int scanNum);




//Function to compute 6 rows of w for a scan
MatrixXd computewScan(double x_bjg, double y_bjg, double z_bjg,
	double w_bjg, double phi_bjg, double K_bjg,
	double x_GPS, double y_GPS, double z_GPS, 
	double w_INS, double phi_INS, double K_INS);


