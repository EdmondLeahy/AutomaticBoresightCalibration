#include "LS.h"


BoresightLS::BoresightLS()
{
}


void Rotation_g2i(double Omega, double Phi, double Kappa, MatrixXd & Rot_g2i) {
	MatrixXd Mw0 = MatrixXd::Zero(3,3);
	MatrixXd Mf0 = MatrixXd::Zero(3,3);
	MatrixXd Mk0 = MatrixXd::Zero(3,3);

	// compute R_g_to_i and return it to the Rot_g2i

	Mw0 << 1, 0, 0,
		0, cosd(Omega), sind(Omega),
		0, -sind(Omega), cosd(Omega);

	Mf0 << cosd(Phi), 0, -sind(Phi),
		0, 1, 0,
		sind(Phi), 0, cosd(Phi);

	Mk0 << cosd(Kappa), sind(Kappa), 0,
		-sind(Kappa), cosd(Kappa), 0,
		0, 0, 1;

	Rot_g2i = Mk0 * Mf0*Mw0;
};

//Function to fill a matrix with info from vector of vectors
void vec2mat(vector<RowVectorXd>& vec, MatrixXd& mat, int cols)
{
	mat = MatrixXd(vec.size(), cols);
	for (int i = 0; i<vec.size(); i++)
	{
		for (int j = 0; j<cols; j++)
		{
			mat(i, j) = vec[i][j];
		}
	}

	return;
}


void BoresightLS::computeA()
{
	A = MatrixXd::Zero(numLidPts, u);
	MatrixXd row = MatrixXd::Zero(1,u);
//	MatrixXd row = MatrixXd::Zero(1,u);
	//Loop for every lidar point observation
	for (int i = 0; i<numLidPts; i++) {
		row = computeAPt(i);
		A.block(i, 0, 1, u) = row;
	}

}

void BoresightLS::computefx(){
	// pass
	VectorXd eop_i = VectorXd::Zero(3);
	MatrixXd R_eop_i = MatrixXd::Zero(3,3);
	VectorXd r_b = VectorXd::Zero(3);
	MatrixXd R_b = MatrixXd::Zero(3,3);
	VectorXd n_i = VectorXd::Zero(3);
	VectorXd p_i = VectorXd::Zero(3);
	int unique_pane, scene;
	double d_i;
	// Initial size of Fx
	fx = VectorXd::Zero(numLidPts);

	// Set the vector, matrix for Boresight (same every iteration)
	r_b(0) = x0(0); //X
	r_b(1) = x0(1); //Y
	r_b(2) = x0(2); //Z
	Rotation_g2i(x0(3),x0(4),x0(5),R_b);

	for(int i=0;i<numLidPts;i++){
		unique_pane = point_details(i,3);
		scene = point_details(i,4);

		// Set the OEP for this point
		eop_i(0) = scene_details(scene,0); //X
		eop_i(1) = scene_details(scene,1); //Y
		eop_i(2) = scene_details(scene,2); //Z
		Rotation_g2i(scene_details(scene,3),scene_details(scene,4),scene_details(scene,5),R_eop_i);

		// Set the vector for plane details
		n_i(0) = plane_details(unique_pane,0); //n1
		n_i(1) = plane_details(unique_pane,1); //n2
		n_i(2) = plane_details(unique_pane,2); //n3
		d_i = plane_details(unique_pane,3); //d

		// Set the point vector for iteration
		p_i(0) = point_details(i,0); //x
		p_i(1) = point_details(i,1); //y
		p_i(2) = point_details(i,2); //z

		// do matrix math

		fx(i) = (eop_i + R_eop_i*r_b + R_eop_i*R_b*p_i).transpose()*n_i + d_i;

	}

}

//Function to compute A
void BoresightLS::computeAandw()
{
	//bs_params: initial approx boresight parameters for entire adjustment. 6 by 1.
	//x_Sjb y_Sjb z_Sjb w_Sb phi_Sb K_Sb

	//plane_details: normals and distance for each plane in vector of vectors. numPlanes by 4.
	//n_xpg, n_ypg, n_zpg, d_p
	//a1, a2, a3, b

	//scene_details: 6 position and orientation parameters from lidar to ground for each scan in vector of vectors. numScans by 6.
	//x_bjg y_bjg z_bjg w_bjg phi_bjg K_bjg
	//X, Y, Z, omega, phi, kappa;

	//point_details format: all point coordinates OBSERVED with lidar in vector of vectors. numLidPts by 3.
	//The same actual point can be observed in diff scenes and have diff x,y,z values.
	//x_sj y_sj z_sj
	//x y z


	MatrixXd A1(0, 0);
	MatrixXd A2(0, 0);
	MatrixXd A3(0, 0);

	MatrixXd w1(0, 0);
	MatrixXd w2(0, 0);
	MatrixXd w3(0, 0);


	//Loop for scans
//	for (int i = 0; i<numScans; i++) {
//		A1.conservativeResize(A1.rows() + 6, u);
//		A1.block(A1.rows() - 6, 0, 6, u) = computeAScan(u, numPlanes, i);
//
//		w1.conservativeResize(w1.rows() + 6, 1);
//		w1.block(w1.rows() - 6, 0, 6, 1) = computewScan(scene_details(i, 0), scene_details(i, 1), scene_details(i, 2),
//			scene_details(i, 3), scene_details(i, 4), scene_details(i, 5),
//			GNSS_INS_data(i, 0), GNSS_INS_data(i, 1), GNSS_INS_data(i, 2),
//			GNSS_INS_data(i, 3), GNSS_INS_data(i, 4), GNSS_INS_data(i, 5));
//	}
//
//	//cout << endl << A1 << endl << endl;
//
//
//	for (int j = 0; j<numPlanes; j++) {
//		A2.conservativeResize(A2.rows() + 1, u);
//		A2.block(A2.rows() - 1, 0, 1, u) = computeAPlane(u, j, plane_details(j, 0), plane_details(j, 1), plane_details(j, 2));
//
//		w2.conservativeResize(w2.rows() + 1, 1);
//		w2.block(w2.rows() - 1, 0, 1, 1) = computewPlane(plane_details(j, 0), plane_details(j, 1), plane_details(j, 2));
//	}
//	//cout << A2 << endl << endl;
//
//
//	int planeID = 0;
//	int scanID = 0;
//	for (int k = 0; k<numLidPts; k++) {
//
//		A3.conservativeResize(A3.rows() + 1, u);
//		w3.conservativeResize(w3.rows() + 1, 1);
//
//		planeID = point_details(k, 3);
//		scanID = point_details(k, 4);
//
//		A3.block(A3.rows() - 1, 0, 1, u) = computeAPt(u, numPlanes, planeID, scanID,
//			bs_params(0, 0), bs_params(1, 0), bs_params(2, 0),
//			bs_params(3, 0), bs_params(4, 0), bs_params(5, 0),
//			plane_details(planeID, 0), plane_details(planeID, 1), plane_details(planeID, 2),
//			scene_details(scanID, 0), scene_details(scanID, 1), scene_details(scanID, 2),
//			scene_details(scanID, 3), scene_details(scanID, 4), scene_details(scanID, 5),
//			point_details(k, 0), point_details(k, 1), point_details(k, 2));
//
//		w3.block(w3.rows() - 1, 0, 1, 1) = computewPt(bs_params(0, 0), bs_params(1, 0), bs_params(2, 0),
//			bs_params(3, 0), bs_params(4, 0), bs_params(5, 0),
//			plane_details(planeID, 0), plane_details(planeID, 1), plane_details(planeID, 2), plane_details(planeID, 3),
//			scene_details(scanID, 0), scene_details(scanID, 1), scene_details(scanID, 2),
//			scene_details(scanID, 3), scene_details(scanID, 4), scene_details(scanID, 5),
//			point_details(k, 0), point_details(k, 1), point_details(k, 2));
//	}
//
//	//for debugging
//	//cout << i << "," << j << "," << k << "\n";
//
//
//	//Combine scan rows, plane rows, and column rows into the full A matrix
//	//cout << A_full.rows() << "," << A_full.cols() << "\n";
//	//cout << A1.rows() << "," << A1.cols() << "\n";
//	//cout << A2.rows() << "," << A2.cols() << "\n";
//	//cout << A3.rows() << "," << A3.cols() << "\n";
//	A_full.block(0, 0, A1.rows(), u) = A1;
//	A_full.block(numScans * 6, 0, A2.rows(), u) = A2;
//	A_full.block(numScans * 6 + numPlanes, 0, A3.rows(), u) = A3;
//
//	A.block(0, 0, A1.rows(), u) = A1;
//	A.block(numScans * 6, 0, A3.rows(), u) = A3;
//
//	H = A2;
//
//	A1.resize(0, 0);
//	A2.resize(0, 0);
//	A3.resize(0, 0);
//
//
//	w_full.block(0, 0, w1.rows(), 1) = w1;
//	w_full.block(numScans * 6, 0, w2.rows(), 1) = w2;
//	w_full.block(numScans * 6 + numPlanes, 0, w3.rows(), 1) = w3;
//
//	w.block(0, 0, w1.rows(), 1) = w1;
//	w.block(numScans * 6, 0, w3.rows(), 1) = w3;
//
//	V = w2;


	return;
}


//Function to compute elements of a rotation matrix, image to object rotation
MatrixXd BoresightLS::RotMatElements(double w, double phi, double K)
{
	MatrixXd Rotw(3, 3);
	MatrixXd Rotphi(3, 3);
	MatrixXd RotK(3, 3);

	Rotw << 1, 0, 0,
		0, cosd(w), sind(w),
		0, -sind(w), cosd(w);

	Rotphi << cosd(phi), 0, -sind(phi),
		0, 1, 0,
		sind(phi), 0, cosd(phi);

	RotK << cosd(K), sind(K), 0,
		-sind(K), cosd(K), 0,
		0, 0, 1;

	MatrixXd RotMat = RotK*Rotphi*Rotw;
	RotMat.transposeInPlace();

	return RotMat;
}



//Function to compute the derivatives of a point equation wrt rotation matrix elements for Rbjg
MatrixXd BoresightLS::PtEqnWrtRotbjg(double x_Sjb, double y_Sjb, double z_Sjb, double w_Sb, double phi_Sb, double K_Sb, double x_sj, double y_sj, double z_sj, double n_xpg, double n_ypg, double n_zpg)
{
	MatrixXd Dbjg1(9, 1);

	MatrixXd RotMat = RotMatElements(w_Sb, phi_Sb, K_Sb);

	double RSb_11 = RotMat(0, 0);
	double RSb_12 = RotMat(0, 1);
	double RSb_13 = RotMat(0, 2);
	double RSb_21 = RotMat(1, 0);
	double RSb_22 = RotMat(1, 1);
	double RSb_23 = RotMat(1, 2);
	double RSb_31 = RotMat(2, 0);
	double RSb_32 = RotMat(2, 1);
	double RSb_33 = RotMat(2, 2);

	Dbjg1(0, 0) = n_xpg*(x_Sjb + RSb_11*x_sj + RSb_12*y_sj + RSb_13*z_sj);
	Dbjg1(1, 0) = n_xpg*(y_Sjb + RSb_21*x_sj + RSb_22*y_sj + RSb_23*z_sj);
	Dbjg1(2, 0) = n_xpg*(z_Sjb + RSb_31*x_sj + RSb_32*y_sj + RSb_33*z_sj);
	Dbjg1(3, 0) = n_ypg*(x_Sjb + RSb_11*x_sj + RSb_12*y_sj + RSb_13*z_sj);
	Dbjg1(4, 0) = n_ypg*(y_Sjb + RSb_21*x_sj + RSb_22*y_sj + RSb_23*z_sj);
	Dbjg1(5, 0) = n_ypg*(z_Sjb + RSb_31*x_sj + RSb_32*y_sj + RSb_33*z_sj);
	Dbjg1(6, 0) = n_zpg*(x_Sjb + RSb_11*x_sj + RSb_12*y_sj + RSb_13*z_sj);
	Dbjg1(7, 0) = n_zpg*(y_Sjb + RSb_21*x_sj + RSb_22*y_sj + RSb_23*z_sj);
	Dbjg1(8, 0) = n_zpg*(z_Sjb + RSb_31*x_sj + RSb_32*y_sj + RSb_33*z_sj);

	return Dbjg1;
}



//Function to compute the derivatives of a point equation wrt rotation matrix elements for RSb
MatrixXd BoresightLS::PtEqnWrtRotSb(double w_bjg, double phi_bjg, double K_bjg, double x_sj, double y_sj, double z_sj, double n_xpg, double n_ypg, double n_zpg)
{
	MatrixXd DSb1(9, 1);

	MatrixXd RotMat = RotMatElements(w_bjg, phi_bjg, K_bjg);

	double Rbjg_11 = RotMat(0, 0);
	double Rbjg_12 = RotMat(0, 1);
	double Rbjg_13 = RotMat(0, 2);
	double Rbjg_21 = RotMat(1, 0);
	double Rbjg_22 = RotMat(1, 1);
	double Rbjg_23 = RotMat(1, 2);
	double Rbjg_31 = RotMat(2, 0);
	double Rbjg_32 = RotMat(2, 1);
	double Rbjg_33 = RotMat(2, 2);

	DSb1(0, 0) = x_sj*(Rbjg_11*n_xpg + Rbjg_21*n_ypg + Rbjg_31*n_zpg);
	DSb1(1, 0) = y_sj*(Rbjg_11*n_xpg + Rbjg_21*n_ypg + Rbjg_31*n_zpg);
	DSb1(2, 0) = z_sj*(Rbjg_11*n_xpg + Rbjg_21*n_ypg + Rbjg_31*n_zpg);
	DSb1(3, 0) = x_sj*(Rbjg_12*n_xpg + Rbjg_22*n_ypg + Rbjg_32*n_zpg);
	DSb1(4, 0) = y_sj*(Rbjg_12*n_xpg + Rbjg_22*n_ypg + Rbjg_32*n_zpg);
	DSb1(5, 0) = z_sj*(Rbjg_12*n_xpg + Rbjg_22*n_ypg + Rbjg_32*n_zpg);
	DSb1(6, 0) = x_sj*(Rbjg_13*n_xpg + Rbjg_23*n_ypg + Rbjg_33*n_zpg);
	DSb1(7, 0) = y_sj*(Rbjg_13*n_xpg + Rbjg_23*n_ypg + Rbjg_33*n_zpg);
	DSb1(8, 0) = z_sj*(Rbjg_13*n_xpg + Rbjg_23*n_ypg + Rbjg_33*n_zpg);

	return DSb1;
}

void BoresightLS::setAdjustmentDetails(MatrixXd point_details_in, MatrixXd plane_details_in, MatrixXd scene_details_in, VectorXd x0_est) {
	point_details = point_details_in;
	plane_details = plane_details_in;
	scene_details = scene_details_in;

	numLidPts = point_details.rows();
	numPlanes = plane_details.rows();
	numScenes = scene_details.rows();

	setX0(x0_est);

	u = 6 + 4*numPlanes;
}

//Function to compute derivatives of rotation matrix elements wrt rotation angles
MatrixXd BoresightLS::RotWrtAngles(double w, double phi, double K)
{
	MatrixXd D = MatrixXd::Zero(9, 3);

	D(0, 1) = -cosd(K)*sind(phi);
	D(0, 2) = -sind(K)*cosd(phi);
	//[0, -cosd(K)*sind(phi), -sind(K)*cosd(phi)]

	D(1, 0) = cosd(K)*cosd(w)*sind(phi) - sind(K)*sind(w);
	D(1, 1) = cosd(K)*cosd(phi)*sind(w);
	D(1, 2) = cosd(K)*cosd(w) - sind(K)*sind(phi)*sind(w);
	//[cosd(K)*cosd(w)*sind(phi) - sind(K)*sind(w), cosd(K)*cosd(phi)*sind(w), cosd(K)*cosd(w) - sind(K)*sind(phi)*sind(w)]

	D(2, 0) = sind(K)*cosd(w) + cosd(K)*sind(phi)*sind(w);
	D(2, 1) = -cosd(K)*cosd(phi)*cosd(w);
	D(2, 2) = cosd(K)*sind(w) + sind(K)*cosd(w)*sind(phi);
	//[sind(K)*cosd(w) + cosd(K)*sind(phi)*sind(w), -cosd(K)*cosd(phi)*cosd(w), cosd(K)*sind(w) + sind(K)*cosd(w)*sind(phi)]

	D(3, 1) = sind(K)*sind(phi);
	D(3, 2) = -cosd(K)*cosd(phi);
	//[0, sind(K)*sind(phi), -cosd(K)*cosd(phi)]

	D(4, 0) = -cosd(K)*sind(w) - sind(K)*cosd(w)*sind(phi);
	D(4, 1) = -sind(K)*cosd(phi)*sind(w);
	D(4, 2) = -sind(K)*cosd(w) - cosd(K)*sind(phi)*sind(w);
	//[ - cosd(K)*sind(w) - sind(K)*cosd(w)*sind(phi), -sind(K)*cosd(phi)*sind(w), - sind(K)*cosd(w) - cosd(K)*sind(phi)*sind(w)]

	D(5, 0) = cosd(K)*cosd(w) - sind(K)*sind(phi)*sind(w);
	D(5, 1) = sind(K)*cosd(phi)*cosd(w);
	D(5, 2) = cosd(K)*cosd(w)*sind(phi) - sind(K)*sind(w);
	//[cosd(K)*cosd(w) - sind(K)*sind(phi)*sind(w), sind(K)*cosd(phi)*cosd(w), cosd(K)*cosd(w)*sind(phi) - sind(K)*sind(w)]

	D(6, 1) = cosd(phi);
	//[0, cosd(phi),  0]

	D(7, 0) = -cosd(phi)*cosd(w);
	D(7, 1) = sind(phi)*sind(w);
	//[ -cosd(phi)*cosd(w),sind(phi)*sind(w), 0]

	D(8, 0) = -cosd(phi)*sind(w);
	D(8, 1) = -cosd(w)*sind(phi);
	//[ -cosd(phi)*sind(w),  -cosd(w)*sind(phi), 0]

	return D;
}



//Function to compute a row of A for a point
//pointNum is the number of a point that matches the point equation
MatrixXd BoresightLS::computeAPt(int pt_index)

//int u, int numPlanes, int planeNum, int scanNum,
//	double x_Sjb, double y_Sjb, double z_Sjb,
//	double w_Sb, double phi_Sb, double K_Sb,
//	double n_xpg, double n_ypg, double n_zpg,
//	double x_bjg, double y_bjg, double z_bjg,
//	double w_bjg, double phi_bjg, double K_bjg,
//	double x_sj, double y_sj, double z_sj)
{
	// EOP of scene:
	int sceneNum = point_details(pt_index,4);
	double x_Sjb = scene_details(sceneNum,0); // X
	double y_Sjb = scene_details(sceneNum,1); // Y
	double z_Sjb = scene_details(sceneNum,2); // Z
	double w_Sb = scene_details(sceneNum,3); // Omega
	double phi_Sb = scene_details(sceneNum,4); // Phi
	double K_Sb = scene_details(sceneNum,5); // Kappa

	// Boresight parameters
	double x_bjg = x0[0]; // x
	double y_bjg = x0[1]; // y
	double z_bjg = x0[2]; // z
	double w_bjg = x0[3]; // omega
	double phi_bjg = x0[4]; // phi
	double K_bjg = x0[5]; // kappa

	// Point parameters
	double x_sj = point_details(pt_index,0); // x
	double y_sj = point_details(pt_index,1); // y
	double z_sj = point_details(pt_index,2); // z

	// Plane parameters (note, we don't need d here as the derivative is 1)
	int planeNum = point_details(pt_index,3);
	double n_xpg = plane_details(planeNum,0); // n1
	double n_ypg = plane_details(planeNum,1); // n2
	double n_zpg = plane_details(planeNum,2); // n3


	// Unknows = [6 boresight, 4*number of planes]
	MatrixXd A_point = MatrixXd::Zero(1, u);


	//6 Boresight parameters
	A_point(0, 0) = (n_ypg*(sind(K_bjg)*cosd(w_bjg) + cosd(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + n_zpg*(sind(K_bjg)*sind(w_bjg)
		- cosd(K_bjg)*cosd(w_bjg)*sind(phi_bjg)) + n_xpg*cosd(K_bjg)*cosd(phi_bjg));


	A_point(0, 1) = (n_ypg*(cosd(K_bjg)*cosd(w_bjg) - sind(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + n_zpg*(cosd(K_bjg)*sind(w_bjg)
		+ sind(K_bjg)*cosd(w_bjg)*sind(phi_bjg)) - n_xpg*sind(K_bjg)*cosd(phi_bjg));


	A_point(0, 2) = (n_xpg*sind(phi_bjg) + n_zpg*cosd(phi_bjg)*cosd(w_bjg) - n_ypg*cosd(phi_bjg)*sind(w_bjg));

	MatrixXd B_PtWrtRot = PtEqnWrtRotSb(w_bjg, phi_bjg, K_bjg, x_sj, y_sj, z_sj, n_xpg, n_ypg, n_zpg);

	MatrixXd B_RotWrtAng = RotWrtAngles(w_Sb, phi_Sb, K_Sb);

	A_point.block(0, 3, 1, 3) = B_PtWrtRot.transpose()*B_RotWrtAng;

	B_PtWrtRot.resize(0, 0);
	B_RotWrtAng.resize(0, 0);


	//4 Plane parameters
	int i = 6 + 4 * planeNum;

	A_point(0, i) = (x_bjg + x_sj*(sind(phi_bjg)*(sind(K_Sb)*sind(w_Sb) - cosd(K_Sb)*cosd(w_Sb)*sind(phi_Sb)) - sind(K_bjg)*cosd(phi_bjg)*(sind(K_Sb)*cosd(w_Sb)
		+ cosd(K_Sb)*sind(phi_Sb)*sind(w_Sb)) + cosd(K_Sb)*cosd(K_bjg)*cosd(phi_Sb)*cosd(phi_bjg)) - y_sj*(sind(K_bjg)*cosd(phi_bjg)*(cosd(K_Sb)*cosd(w_Sb)
			- sind(K_Sb)*sind(phi_Sb)*sind(w_Sb)) - sind(phi_bjg)*(cosd(K_Sb)*sind(w_Sb) + sind(K_Sb)*cosd(w_Sb)*sind(phi_Sb))
			+ cosd(K_bjg)*sind(K_Sb)*cosd(phi_Sb)*cosd(phi_bjg)) + z_sj*(cosd(K_bjg)*cosd(phi_bjg)*sind(phi_Sb) + cosd(phi_Sb)*cosd(w_Sb)*sind(phi_bjg)
				+ sind(K_bjg)*cosd(phi_Sb)*cosd(phi_bjg)*sind(w_Sb)) + z_Sjb*sind(phi_bjg) + x_Sjb*cosd(K_bjg)*cosd(phi_bjg) - y_Sjb*sind(K_bjg)*cosd(phi_bjg));

	A_point(0, i + 1) = (y_bjg + x_Sjb*(sind(K_bjg)*cosd(w_bjg) + cosd(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + y_Sjb*(cosd(K_bjg)*cosd(w_bjg)
		- sind(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + x_sj*((sind(K_Sb)*cosd(w_Sb) + cosd(K_Sb)*sind(phi_Sb)*sind(w_Sb))*(cosd(K_bjg)*cosd(w_bjg)
			- sind(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + cosd(K_Sb)*cosd(phi_Sb)*(sind(K_bjg)*cosd(w_bjg) + cosd(K_bjg)*sind(phi_bjg)*sind(w_bjg))
			- cosd(phi_bjg)*sind(w_bjg)*(sind(K_Sb)*sind(w_Sb) - cosd(K_Sb)*cosd(w_Sb)*sind(phi_Sb))) - y_sj*(sind(K_Sb)*cosd(phi_Sb)*(sind(K_bjg)*cosd(w_bjg)
				+ cosd(K_bjg)*sind(phi_bjg)*sind(w_bjg)) - (cosd(K_Sb)*cosd(w_Sb) - sind(K_Sb)*sind(phi_Sb)*sind(w_Sb))*(cosd(K_bjg)*cosd(w_bjg)
					- sind(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + cosd(phi_bjg)*sind(w_bjg)*(cosd(K_Sb)*sind(w_Sb) + sind(K_Sb)*cosd(w_Sb)*sind(phi_Sb)))
		- z_sj*(cosd(phi_Sb)*sind(w_Sb)*(cosd(K_bjg)*cosd(w_bjg) - sind(K_bjg)*sind(phi_bjg)*sind(w_bjg)) - sind(phi_Sb)*(sind(K_bjg)*cosd(w_bjg)
			+ cosd(K_bjg)*sind(phi_bjg)*sind(w_bjg)) + cosd(phi_Sb)*cosd(phi_bjg)*cosd(w_Sb)*sind(w_bjg)) - z_Sjb*cosd(phi_bjg)*sind(w_bjg));

	A_point(0, i + 2) = (z_bjg + x_Sjb*(sind(K_bjg)*sind(w_bjg) - cosd(K_bjg)*cosd(w_bjg)*sind(phi_bjg)) + y_Sjb*(cosd(K_bjg)*sind(w_bjg)
		+ sind(K_bjg)*cosd(w_bjg)*sind(phi_bjg)) + x_sj*((sind(K_Sb)*cosd(w_Sb) + cosd(K_Sb)*sind(phi_Sb)*sind(w_Sb))*(cosd(K_bjg)*sind(w_bjg)
			+ sind(K_bjg)*cosd(w_bjg)*sind(phi_bjg)) + cosd(K_Sb)*cosd(phi_Sb)*(sind(K_bjg)*sind(w_bjg) - cosd(K_bjg)*cosd(w_bjg)*sind(phi_bjg))
			+ cosd(phi_bjg)*cosd(w_bjg)*(sind(K_Sb)*sind(w_Sb) - cosd(K_Sb)*cosd(w_Sb)*sind(phi_Sb))) + y_sj*((cosd(K_Sb)*cosd(w_Sb)
				- sind(K_Sb)*sind(phi_Sb)*sind(w_Sb))*(cosd(K_bjg)*sind(w_bjg) + sind(K_bjg)*cosd(w_bjg)*sind(phi_bjg))
				- sind(K_Sb)*cosd(phi_Sb)*(sind(K_bjg)*sind(w_bjg) - cosd(K_bjg)*cosd(w_bjg)*sind(phi_bjg)) + cosd(phi_bjg)*cosd(w_bjg)*(cosd(K_Sb)*sind(w_Sb)
					+ sind(K_Sb)*cosd(w_Sb)*sind(phi_Sb))) + z_sj*(sind(phi_Sb)*(sind(K_bjg)*sind(w_bjg) - cosd(K_bjg)*cosd(w_bjg)*sind(phi_bjg))
						- cosd(phi_Sb)*sind(w_Sb)*(cosd(K_bjg)*sind(w_bjg) + sind(K_bjg)*cosd(w_bjg)*sind(phi_bjg))
						+ cosd(phi_Sb)*cosd(phi_bjg)*cosd(w_Sb)*cosd(w_bjg)) + z_Sjb*cosd(phi_bjg)*cosd(w_bjg));

	A_point(0, i + 3) = 1;


	return A_point;
}


MatrixXd BoresightLS::computewPt(double x_Sjb, double y_Sjb, double z_Sjb,
	double w_Sb, double phi_Sb, double K_Sb,
	double n_xpg, double n_ypg, double n_zpg, double d_p,
	double x_bjg, double y_bjg, double z_bjg,
	double w_bjg, double phi_bjg, double K_bjg,
	double x_sj, double y_sj, double z_sj)
{
	MatrixXd w3(1, 1);


	w3(0, 0) = (d_p + n_ypg*(y_bjg + x_Sjb*(sin(K_bjg)*cos(w_bjg) + cos(K_bjg)*sin(phi_bjg)*sin(w_bjg)) + y_Sjb*(cos(K_bjg)*cos(w_bjg)
		- sin(K_bjg)*sin(phi_bjg)*sin(w_bjg)) + x_sj*((sin(K_Sb)*cos(w_Sb) + cos(K_Sb)*sin(phi_Sb)*sin(w_Sb))*(cos(K_bjg)*cos(w_bjg)
			- sin(K_bjg)*sin(phi_bjg)*sin(w_bjg)) + cos(K_Sb)*cos(phi_Sb)*(sin(K_bjg)*cos(w_bjg) + cos(K_bjg)*sin(phi_bjg)*sin(w_bjg))
			- cos(phi_bjg)*sin(w_bjg)*(sin(K_Sb)*sin(w_Sb) - cos(K_Sb)*cos(w_Sb)*sin(phi_Sb)))
		- y_sj*(sin(K_Sb)*cos(phi_Sb)*(sin(K_bjg)*cos(w_bjg) + cos(K_bjg)*sin(phi_bjg)*sin(w_bjg)) - (cos(K_Sb)*cos(w_Sb)
			- sin(K_Sb)*sin(phi_Sb)*sin(w_Sb))*(cos(K_bjg)*cos(w_bjg) - sin(K_bjg)*sin(phi_bjg)*sin(w_bjg)) + cos(phi_bjg)*sin(w_bjg)*(cos(K_Sb)*sin(w_Sb)
				+ sin(K_Sb)*cos(w_Sb)*sin(phi_Sb))) - z_sj*(cos(phi_Sb)*sin(w_Sb)*(cos(K_bjg)*cos(w_bjg) - sin(K_bjg)*sin(phi_bjg)*sin(w_bjg))
					- sin(phi_Sb)*(sin(K_bjg)*cos(w_bjg) + cos(K_bjg)*sin(phi_bjg)*sin(w_bjg)) + cos(phi_Sb)*cos(phi_bjg)*cos(w_Sb)*sin(w_bjg))
		- z_Sjb*cos(phi_bjg)*sin(w_bjg)) + n_xpg*(x_bjg + x_sj*(sin(phi_bjg)*(sin(K_Sb)*sin(w_Sb) - cos(K_Sb)*cos(w_Sb)*sin(phi_Sb))
			- sin(K_bjg)*cos(phi_bjg)*(sin(K_Sb)*cos(w_Sb) + cos(K_Sb)*sin(phi_Sb)*sin(w_Sb)) + cos(K_Sb)*cos(K_bjg)*cos(phi_Sb)*cos(phi_bjg))
			- y_sj*(sin(K_bjg)*cos(phi_bjg)*(cos(K_Sb)*cos(w_Sb) - sin(K_Sb)*sin(phi_Sb)*sin(w_Sb)) - sin(phi_bjg)*(cos(K_Sb)*sin(w_Sb)
				+ sin(K_Sb)*cos(w_Sb)*sin(phi_Sb)) + cos(K_bjg)*sin(K_Sb)*cos(phi_Sb)*cos(phi_bjg)) + z_sj*(cos(K_bjg)*cos(phi_bjg)*sin(phi_Sb)
					+ cos(phi_Sb)*cos(w_Sb)*sin(phi_bjg) + sin(K_bjg)*cos(phi_Sb)*cos(phi_bjg)*sin(w_Sb)) + z_Sjb*sin(phi_bjg)
			+ x_Sjb*cos(K_bjg)*cos(phi_bjg) - y_Sjb*sin(K_bjg)*cos(phi_bjg)) + n_zpg*(z_bjg + x_Sjb*(sin(K_bjg)*sin(w_bjg)
				- cos(K_bjg)*cos(w_bjg)*sin(phi_bjg)) + y_Sjb*(cos(K_bjg)*sin(w_bjg) + sin(K_bjg)*cos(w_bjg)*sin(phi_bjg))
				+ x_sj*((sin(K_Sb)*cos(w_Sb) + cos(K_Sb)*sin(phi_Sb)*sin(w_Sb))*(cos(K_bjg)*sin(w_bjg) + sin(K_bjg)*cos(w_bjg)*sin(phi_bjg))
					+ cos(K_Sb)*cos(phi_Sb)*(sin(K_bjg)*sin(w_bjg) - cos(K_bjg)*cos(w_bjg)*sin(phi_bjg)) + cos(phi_bjg)*cos(w_bjg)*(sin(K_Sb)*sin(w_Sb)
						- cos(K_Sb)*cos(w_Sb)*sin(phi_Sb))) + y_sj*((cos(K_Sb)*cos(w_Sb) - sin(K_Sb)*sin(phi_Sb)*sin(w_Sb))*(cos(K_bjg)*sin(w_bjg)
							+ sin(K_bjg)*cos(w_bjg)*sin(phi_bjg)) - sin(K_Sb)*cos(phi_Sb)*(sin(K_bjg)*sin(w_bjg) - cos(K_bjg)*cos(w_bjg)*sin(phi_bjg))
							+ cos(phi_bjg)*cos(w_bjg)*(cos(K_Sb)*sin(w_Sb) + sin(K_Sb)*cos(w_Sb)*sin(phi_Sb))) + z_sj*(sin(phi_Sb)*(sin(K_bjg)*sin(w_bjg)
								- cos(K_bjg)*cos(w_bjg)*sin(phi_bjg)) - cos(phi_Sb)*sin(w_Sb)*(cos(K_bjg)*sin(w_bjg) + sin(K_bjg)*cos(w_bjg)*sin(phi_bjg))
								+ cos(phi_Sb)*cos(phi_bjg)*cos(w_Sb)*cos(w_bjg)) + z_Sjb*cos(phi_bjg)*cos(w_bjg)));


	return w3;
}





