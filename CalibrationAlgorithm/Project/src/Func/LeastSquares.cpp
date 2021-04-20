#include "LeastSquares.h"



LeastSquares::LeastSquares()
{
	num_iter = 0;
	n = 0;
	u = 0;
	clear();
	isdebug = false;
	isquiet = false;
	snoop=false;
	aprior = 1;
	apost = -1;

}
void LeastSquares::clear()
{
	A = MatrixXd::Zero(1,1);
	Cl = MatrixXd::Zero(1, 1);
	Cxh = MatrixXd::Zero(1,1);
	xh = VectorXd::Zero(1);
	w = VectorXd::Zero(1);
	fx = VectorXd::Zero(1);
	l = VectorXd::Zero(1);
	isdebug = false;
	isquiet = false;
}
;

void LeastSquares::setIter(int num_iter_temp)
{
	num_iter = num_iter_temp;
}


void LeastSquares::setTol(double tol_temp)
{
	tol = tol_temp;
}

void LeastSquares::setGlobalSnoopConfidence(double confidence)
{
	globalSnoopConf = confidence;
}

void LeastSquares::setLocalSnoopConfidence(double confidence)
{
	localSnoopConf = confidence;
}

void LeastSquares::setAPrior(double prior)
{
	aprior = prior;
}

void LeastSquares::setL(const VectorXd & l_temp)
{
	l = l_temp;
}

void LeastSquares::setX0(const VectorXd & x0_temp)
{
	x0 = x0_temp;
}


void LeastSquares::setX0(double x, double y, double z, double te)
{
	x0.resize(4);
	x0(0) = x;
	x0(1) = y;
	x0(2) = z;
	x0(3) = te;
}

void LeastSquares::setxh(const VectorXd & xh_temp)
{

}

void LeastSquares::setCl(const MatrixXd & Cl_diag)
{
	Cl = Cl_diag.asDiagonal();
}

void LeastSquares::setP(const VectorXd diag_p)
{
	P = diag_p.asDiagonal();
}

void LeastSquares::setA(const MatrixXd & A_temp)
{
	A = A_temp;
}

void LeastSquares::setfx(const MatrixXd & fx_temp)
{
	fx = fx_temp;
}

VectorXd LeastSquares::getxh()
{
	return xh;
}

VectorXd LeastSquares::getx0()
{
	return x0;
}

VectorXd LeastSquares::getlh()
{
	return lh;
}

VectorXd LeastSquares::getd()
{
	return d;
}

VectorXd LeastSquares::getw()
{
	return w;
}


double LeastSquares::getAPost()
{
	return apost;
}

MatrixXd LeastSquares::getP()
{
	return P;
}

MatrixXd LeastSquares::getA()
{
	return A;
}

MatrixXd LeastSquares::getN()
{
	return N;
}

MatrixXd LeastSquares::getNi()
{
	return Ni;
}

MatrixXd LeastSquares::getCl()
{
	return Cl;
}

MatrixXd LeastSquares::getClh()
{	
	return Clh;
}

MatrixXd LeastSquares::getCvh()
{
	return Cvh;
}

MatrixXd LeastSquares::getCxh()
{
	return Cxh;
}

VectorXd LeastSquares::getv()
{
	return v;
}

VectorXd LeastSquares::getL()
{
	return l;
}

VectorXd LeastSquares::getfx()
{
	return fx;
}



MatrixXd LeastSquares::getDataSnoop()
{
	return y;
}

MatrixXd LeastSquares::getRho()
{
	return rho;
}

MatrixXd LeastSquares::getErrorEllipse()
{
	return ellipseOrientation;
}

MatrixXd LeastSquares::getSemiAxis()
{
	return errorEllipse;
}


void LeastSquares::quiet()
{
	isquiet = true;
}

void LeastSquares::debug()
{
	isdebug = true;
}

void LeastSquares::computeLS()
{
	//Compute non-iterative LS
	//Set initial parameters
	cout << "\nRunning linear LeastSquares estimation......";
	computeA();
	n = A.rows();
	u = A.cols();
	r = computeRank(A);
	computeP();

//	cout << "A:\n" << A << endl;
//	cout << "Cl:\n" << Cl << endl;

//	cout << "Shape of A: " << A.rows() << " X " << A.cols() << endl;
//	print_matrix(Cl);
//	print_matrix(A);
//	print_matrix(P);
//	cout << "\n\nn:  " << n << endl;
//	cout << "u:  " << u << endl;
//	cout << "r:  " << r << endl;

	computeDelta();
	updateUnknowns();
	computeResiduals();
	computeaPost();
//	computeClh();
//	computeCxh();
//	updateObservations();

	if (isquiet != true && isdebug) {
		//TESTING
//		cout << "L:\n" << l << endl;
//		cout << "v:\n" << v << endl;
//		cout << "Fx:\n" << fx << endl;
		cout << "d:\n" << d << endl;
//		cout << "w:\n" << w << endl;
//		cout << "A:\n" << A << endl;
//		cout << "P:\n" << P << endl;
//		cout << "Cl:\n" << Cl << endl;
//		cout << "N:\n" << N << endl;
//		cout << "U:\n" << U << endl;
//		cout << "n: " << n << endl;
//		cout << "u: " << u << endl;
	}
	cout << "FINISHED\n";

}

void LeastSquares::iterate()
{

	fprintf(stdout,"\nRunning Non-Linear LeastSquares estimation for max %i iterations......\n",num_iter);
	//Need to set some initial things:
	computeP();
	n = l.rows();
	u = x0.rows();

	bool Delta = false;
	int i = 0;
	while ((!Delta) && (i < num_iter)) {
		//update iteration counter
		i++;
		if (isquiet != true && isdebug) {

			cout << "-------------------------------------------\n" << endl;
			cout << "START ITERATION: " << i << endl;
			cout << "x0:\n" << x0 << endl;
		}

		//compute design matrix for LS Calculations
		computeA();	

		//ENDTESTING
		//compute the Delta
		computeDelta();

		//update the obs and unkns
		updateUnknowns();

		//Check the tolerance
		if (getMaxDelta() <= tol) {
			Delta = true;
		}

		if (isquiet != true && isdebug) {
			//TESTING
			cout << "L:\n" << l << endl;
			cout << "v:\n" << v << endl;
			cout << "Fx:\n" << fx << endl;
			cout << "d:\n" << d << endl;
			cout << "w:\n" << w << endl;
			cout << "A:\n" << A << endl;
			cout << "P:\n" << P << endl;
			cout << "Cl:\n" << Cl << endl;
			cout << "N:\n" << N << endl;
			cout << "U:\n" << U << endl;
			cout << "n: " << n << endl;
			cout << "u: " << u << endl;
		}
	}
	cout << "FINISHED\n";
	cout << "Computation took: " << i << " iterations \n" << endl;

	
	computeResiduals();//vh
//	updateObservations();//lh
//	computeaPost();//Apost
//	computeCxh();//Cx
//
//
//	computeClh();//Clh
//	computeCvh();
//	computeErrorEllipse(Cxh);
//	if (snoop){
//		dataSnoop();
//	}



}

double LeastSquares::getMaxDelta()
{
	double max = 0.0;

	for (int i = 0; i < d.size(); i++) {
		if (abs(d(i)) > max) {
			max = abs(d(i));
		}

	}
	if (isquiet != true) { cout << "Max Delta for it: " << max << endl; }
	cout << '\n' << d << endl;
	return max;
}

void LeastSquares::computeaPost()
{
	VectorXd temp;
	temp = (v.transpose() * P * v) / (n - r);
	apost = pow(temp[0], 0.5);
}

double LeastSquares::computeRank(MatrixXd &R)
{
	double rank;
	FullPivLU<MatrixXd> luM(R);
	rank = luM.rank();
	return rank;

}

MatrixXd LeastSquares::computeInverse(MatrixXd & M)
{
	MatrixXd temp_I;
	double rows = M.rows();
	double cols = M.cols(); 
	double rank = computeRank(M);;
	if (rows == cols) {
		if (abs(M.determinant()) > 1 * pow(10, -20)){
			//cout << "Invertable matrix. General inverse method applied" << temp_I << endl;
			temp_I = M.inverse();
			goto end;
		}
		else{
			cout << "Matrix is not invertable. Determinant = " << abs(M.determinant())  << ". Computing normal Psuedo Inverse." << endl;
			temp_I = computeNPI(M, rank);
		}
	}
	//Find the rank
	//Find out if LPI or RPI
	else if (rank == rows) { temp_I = computeRPI(M); }
	else if (rank == cols) { temp_I = computeLPI(M); }
	else { temp_I = computeNPI(M, rank); }

	end:
	return temp_I;
}

MatrixXd LeastSquares::computeRPI(MatrixXd & M)
{
	MatrixXd RPI;
	RPI = M.transpose()*(M*M.transpose()).inverse();
	return RPI;
}

MatrixXd LeastSquares::computeLPI(MatrixXd & M)
{
	MatrixXd LPI;
	LPI = (A.transpose()*A).inverse() * A.transpose();
	return LPI;
}

MatrixXd LeastSquares::computeGI(MatrixXd & M, double rank)
{
	double rows = M.rows();
	double c = M.cols();
	MatrixXd GI = MatrixXd::Zero(rows, c);
	MatrixXd B = M.block(0, 0, rank, rank);
	GI.block(0, 0, rank, rank) = B.inverse();
	return GI;
}

MatrixXd LeastSquares::computeGPI(MatrixXd & M, double rank)
{
	MatrixXd Y = M*M.transpose();
	MatrixXd G = M.transpose() * M;
	MatrixXd GPI = M.transpose() * computeGI(Y,rank)*M*(computeGI(G,rank)) * M.transpose();
	return GPI;
}

MatrixXd LeastSquares::computeNPI(MatrixXd & M, double rank)
{
	MatrixXd Y = (M * M.transpose());
	MatrixXd NPI = M.transpose() * computeGI(Y, rank);
	return NPI;
}

double LeastSquares::computeNorm(MatrixXd & M)
{
	double rows = M.rows();
	double cols = M.cols();

	double norm;
	double sum = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			sum = sum + pow(M(i, j),2);
		}
	}
	norm = sqrt(sum);
	return norm;

}

double LeastSquares::computeCondition(MatrixXd & M)
{
	MatrixXd inv_M = computeInverse(M);
	double cond = computeNorm(M) * computeNorm(inv_M);
	return cond;
}

void LeastSquares::computeP()
{
	if (P.rows() < 1){
		P = pow(aprior,2) * Cl.inverse();
	}
}

void LeastSquares::computeU()
{	
	computeMisclosure();
	U = A.transpose() * P * w;
}

void LeastSquares::computeN()
{
	N = A.transpose() * P * A;
//	print_matrix(N);
	computeNi();

}

void LeastSquares::computeNi()
{
	Ni = computeInverse(N);
}

void LeastSquares::updateObservations()
{	
	lh = l + v;
}

void LeastSquares::updateUnknowns()
{
	xh = x0 + d;
	x0 = xh;
}

void LeastSquares::computeResiduals()
{

	v = A * d + w;
}

void LeastSquares::computeMisclosure()
{
	computefx();
	w = -fx;
}

void LeastSquares::computeDelta()
{
	//d = VectorXd::Zero(u);
	computeN();
	computeU();
	d = Ni * U;

}


void LeastSquares::computeClh()
{
	Clh = apost * A * Ni * A.transpose();
}

void LeastSquares::computeCvh()
{
//	Cvh = apost * P.inverse() - Clh;
	Cvh = Clh - A*Cxh*A.transpose();

}

void LeastSquares::computeCxh()
{
	Cxh = apost * Ni;
}

void LeastSquares::dataSnoop()
{
	
	//Global Testing
	VectorXd gt;
	gt = (v.transpose() * P * v) / aprior;
	cout << gt << endl;
	if ((-1* globalSnoopConf) <= gt(0,0) && gt(0,0) <= globalSnoopConf) {
		fprintf(stdout, "The dataset PASSES requirements. The value %f is smaller than %f\n\n", gt(0, 0), globalSnoopConf);
	}
	else {
		fprintf(stdout, "The dataset FAILS requirements. The value %f is larger than %f\n\n", gt(0, 0), globalSnoopConf);
		cout << "Starting local testing\n";
		//Local test
		//We need the Q matrix
		MatrixXd Qv = P.inverse() - A*(Ni)*A.transpose();
		//we need a vector of the failig residuals
		vector <int> blunders;
		y = MatrixXd::Zero(Qv.rows(), 2);
		for (int i = 0; i < Qv.rows(); i++) {
			y(i, 0) = v[i] / (sqrt(aprior) * sqrt(Qv(i, i)));
			if ((-1 * localSnoopConf) <= y(i, 0) && y(i, 0) <= localSnoopConf) {
				y(i, 1) = 1;
			}
			else {
				//cout << "Observation " << i << " fails requirements\n";
				blunders.push_back(i);
				y(i, 1) = 0;
			}
		}
		//We need to check the correlations
		rho = MatrixXd::Zero(blunders.size(), blunders.size());
		for (int j = 0; j < int(blunders.size()); j++) {
			for (int k = 0; k < int(blunders.size()); k++) {
				rho(j, k) = Qv(j, k) / (sqrt(Qv(j, j))*sqrt(Qv(k, k)));
				if (rho(j, k) > 1.0*pow(10, -4) && rho(j, k) < 1) { continue; }
			}
		}
		
	}
}



void LeastSquares::computefx()
{
	//This must be written before running the program. 
	//This is not a general fuction
}

void LeastSquares::computeA()
{
	//This must be written before running the program. 
	//This is not a general fuction
}

double LeastSquares::getTol() {
	return tol;
}

double LeastSquares::getIter() {
	return num_iter;
}


double LeastSquares::dist(double xi, double yi, double xj, double yj)
{
	double dist = sqrt(pow(xi - xj, 2) + pow(yi - yj, 2));
	return dist;
}

double LeastSquares::dist(double i, double j)
{
	double dist = sqrt(pow(i, 2) + pow(j, 2));
	return dist;
}

double LeastSquares::dist(double xi, double yi, double zi, double xj, double yj, double zj)
{
	double dist = sqrt(pow(xi - xj, 2) + pow(yi - yj, 2) + pow(zi - zj, 2));
	return dist;
}

void LeastSquares::computeErrorEllipse(MatrixXd Cx_error)
{
	ellipseOrientation = MatrixXd::Zero(2, 2);
	errorEllipse = MatrixXd::Zero(2, 2);
	for (int k = 0; k<2; k++) {
		MatrixXd QP1 = Cx_error.block(k * 2, k * 2, 2, 2);
		
		errorEllipse(k, 0) = QP1.eigenvalues()[0].real();
		errorEllipse(k, 1) = QP1.eigenvalues()[1].real();

		//Orientations
		double theta_max = atan((errorEllipse(k, 0) - QP1(0, 0) / QP1(0, 1)));
		double theta_min = atan((errorEllipse(k, 1) - QP1(0, 0) / QP1(0, 1)));

		ellipseOrientation(k, 0) = theta_max*RAD2DEG;
		ellipseOrientation(k, 1) = theta_min*RAD2DEG;


		//semi axis is sqrt(lambda)
		for (int j = 0; j < 2; j++) {
			errorEllipse(k, j) = sqrt(errorEllipse(k, j)); // in arcdeg
		}

	}



}


void print_matrix(MatrixXd print_mat)
{
	cout << "\n";
	for (int i = 0; i < print_mat.rows(); i++)
	{
		for (int j = 0; j < print_mat.cols(); j++)
		{
			printf("\t %0.10f ", print_mat(i,j));
		}
		printf("\n");

	}
}
