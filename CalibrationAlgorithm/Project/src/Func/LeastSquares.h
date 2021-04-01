#ifndef LEASTSQUARES_H
#define LEASTSQUARES_H

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <map>

using namespace Eigen;
using namespace std;

class LeastSquares
{
public:
	const double PI = (3.141592653589793238463);
	const double RAD2ARC = (3600 * 180 / PI);
	const double RAD2DEG = (180 / PI);
	LeastSquares();
	void clear();
	//Setters
	void setIter(int num_iter_temp);
	void setTol(double tol_temp);
	void setGlobalSnoopConfidence(double confidence);
	void setLocalSnoopConfidence(double confidence);
	void setAPrior(double prior);
	void setL(const VectorXd & l_temp);
	void setX0(const VectorXd & x0_temp);
	void setX0(double x, double y, double z, double te);
	void setxh(const VectorXd & xh_temp);
	void setCl(const MatrixXd & Cl_diag);
	void setP(const VectorXd diag_p);
	void setA(const MatrixXd &A_temp);
	void setfx(const MatrixXd &fx_temp);
	//Getters
	VectorXd getxh();
	VectorXd getx0();
	VectorXd getlh();
	VectorXd getd();
	VectorXd getw();
	double getAPost();
	MatrixXd getP();
	MatrixXd getA();
	MatrixXd getN();
	MatrixXd getNi();
	MatrixXd getCl();
	MatrixXd getClh();
	MatrixXd getCvh();
	MatrixXd getCxh();
	VectorXd getv();
	VectorXd getL();
	VectorXd getfx();
	MatrixXd getDataSnoop();
	MatrixXd getRho();
	MatrixXd getErrorEllipse();
	MatrixXd getSemiAxis();
	double getTol();
	double getIter();

	//distance

	double dist(double xi, double yi, double zi, double xj, double yj, double zj);
	double dist(double xi, double yi, double xj, double yj);
	double dist(double i, double j);


	//ENGO419 Lab2 necessities
	double computeRank(MatrixXd &R);
	double computeNorm(MatrixXd &M);
	double computeCondition(MatrixXd &M);
	MatrixXd computeInverse(MatrixXd &M);
	MatrixXd computeRPI(MatrixXd &M);
	MatrixXd computeLPI(MatrixXd &M);
	MatrixXd computeGI(MatrixXd &M, double rank);
	MatrixXd computeGPI(MatrixXd &M, double rank);
	MatrixXd computeNPI(MatrixXd &M, double rank);

	//Set the mode to quiet or debug
	void quiet();
	void debug();

	void computeLS();
	void iterate();

	void computeMisclosure();
	void computeErrorEllipse(MatrixXd Cx_error);


	void dataSnoop();


	std::map<int, int> vars;
	std::map<char, int>::iterator it;
	MatrixXd A, Cl, P, N, U, Ni, errorEllipse, ellipseOrientation, rho;
	VectorXd l, lh, x0, xh, fx, d, w, v;
	bool isquiet, isdebug, snoop;

	int num_iter, n, u, r;

protected:




private:

	double tol, aprior, apost, localSnoopConf, globalSnoopConf;

	
	MatrixXd Clh, Cxh, Cvh, y;

	double getMaxDelta();
	void computeaPost();

	void computeP();
	void computeU();
	void computeN();
	void computeNi();

	void updateObservations();
	void updateUnknowns();

	void computeResiduals();
	void computeDelta();

	void computeClh();
	void computeCvh();
	void computeCxh();



	virtual void computefx();
	virtual void computeA();

};
void print_matrix(MatrixXd print_mat);




#endif
