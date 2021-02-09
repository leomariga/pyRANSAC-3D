#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
#include <random>

/*

Author: Guilherme Ferrari Fortino

Adaptation from https://github.com/jczamorac/Tracking_RANSAC

*/

extern "C" {

#define M_PI 3.14159265358979323846

void swap(int &a, int &b){
	int z;
	z = a;
	a = b;
	b = z;
	}

void getPDF(double* charge, double Tcharge, int size, double* PDF){
	int loop;
	for(loop = 0; loop < size; loop++) PDF[loop] = (double) charge[loop]/Tcharge;
}

void get_random(int &ind1, int &ind2, double (*data)[3], double* charge, double* PDF, double TCharge, double AvgCharge, double TwiceAvCharge, int mode, int size){
	if (mode == 0){
	// Random sampling
		ind1 = rand() % size;
		ind2 = rand() % size;
		while (ind1 == ind2) ind2 = rand() % size;
	}
	else if (mode == 1){
	// Gaussian sampling
		std::random_device rd; 
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.0, 1.0);
		int loop;
		double dist = 0;
		double sigma = 30.0;
		double y = 0;
		double gauss = 0;
		int counter = 0;
		double dif[3];
		double P2[3];
		double P1[3];
		ind1 = rand() % size;
		do{
			ind2 = rand() % size;
			for(loop = 0; loop < 3; loop++) P1[loop] = data[ind1][loop];
			for(loop = 0; loop < 3; loop++) P2[loop] = data[ind2][loop];
			for(loop = 0; loop < 3; loop++) dif[loop] = P2[loop] - P1[loop];
			dist = sqrt(pow(dif[0], 2.0) + pow(dif[1], 2.0) + pow(dif[2], 2.0));
			gauss = 1.0*exp(-1.0*pow(dist/sigma,2.0));
			y = dis(gen);
			counter++;
			if(counter>20 && ind2!=ind1) break;
			} while(ind1 == ind2 || y > gauss);
	}
	else if (mode == 2){
		// Weighted sampling

		std::random_device rd; 
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.0, 1.0);

		bool cond = false;
		int counter = 0;
		double w2;

		ind1 = rand() % size;
		do{
		counter++;
		if(counter>30 && ind2!=ind1) break;
		ind2 = rand() % size;
		cond = false;
		w2 = dis(gen)*TwiceAvCharge;
		if(PDF[ind2]>=w2) cond = true;
		} while(ind2==ind1 || cond==false);
	}

	else if(mode == 3){
		// Weighted sampling + Gauss dist.
		std::random_device rd; 
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.0, 1.0);
		bool cond = false;
		double dist = 0;
		double sigma = 30.0;
		double y = 0;
		double gauss = 0;
		int counter = 0;
		double dif[3];
		double P2[3];
		double P1[3];
		double w2;
		int loop;
		ind1 = rand() % size;
		do{
			ind2 = rand() % size;
			for(loop = 0; loop < 3; loop++) P1[loop] = data[ind1][loop];
			for(loop = 0; loop < 3; loop++) P2[loop] = data[ind2][loop];
			for(loop = 0; loop < 3; loop++) dif[loop] = P2[loop] - P1[loop];
			dist = sqrt(pow(dif[0], 2.0) + pow(dif[1], 2.0) + pow(dif[2], 2.0));
			gauss = 1.0*exp(-1.0*pow(dist/sigma,2));
			y = dis(gen);
			counter++;
			if(counter>20 && ind2!=ind1) break;
			cond = false;
			w2 = dis(gen)*TwiceAvCharge;
			if(PDF[ind2] >= w2) cond = true;
			else{
				w2 = 1.0;
				cond = true;
			}
		} while(ind2 == ind1 || cond == false || y > gauss);
	}
}

int Ransac(double (*data)[3], double *versor, double* pb, int *inliers, double* charge, int number_it, double min_dist, int size, int mode, int selection_model){
	int indice1, indice2, loop, j;
	std::vector <int> parcial;
	double parcial_versor[3];
	std::vector <int> best;
	double distancia, norma1, norma2;
	int qtd = 0;
	double point_A[3];
	double point_B[3];
	double aux[3];
	double cross[3];
	double TCharge = 0;
	double PDF[size];
	double AvgCharge, TwiceAvCharge;

	if (mode == 2 || mode == 3){
		for (loop = 0; loop < size; loop++) TCharge += charge[loop];
		AvgCharge = (double) TCharge/size;
		TwiceAvCharge = (double) 2*AvgCharge;
		getPDF(charge, TCharge, size, PDF);
	}

	for(int i = 0; i <= number_it; i++){
		get_random(indice1, indice2, data, charge, PDF, TCharge, AvgCharge, TwiceAvCharge, mode, size);
		
		for(loop = 0; loop < 3; loop++) point_A[loop] = data[indice1][loop];
		
		for(loop = 0; loop < 3; loop++) point_B[loop] = data[indice2][loop];
		
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) point_B[loop] - point_A[loop];
		
		norma1 = (double)sqrt((double)pow(parcial_versor[0], 2) + (double)pow(parcial_versor[1], 2) + (double)pow(parcial_versor[2], 2));
		
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) parcial_versor[loop]/norma1;
		
		for(j = 0; j < size; j++){
			for(loop = 0; loop < 3; loop++) aux[loop] = point_A[loop] - data[j][loop];
			
			cross[0] = (double) parcial_versor[1]*aux[2] - parcial_versor[2]*aux[1];
			cross[1] = (double) -(parcial_versor[0]*aux[2] - parcial_versor[2]*aux[0]);
			cross[2] = (double) parcial_versor[0]*aux[1] - parcial_versor[1]*aux[0];
			
			norma2 = sqrt(pow(cross[0], 2.0) + pow(cross[1], 2.0) + pow(cross[2], 2.0));
			if(fabs(norma2) <= min_dist) parcial.push_back(j); 
		}
		if(parcial.size() > best.size()){ // 
			best.clear(); // Limpa o vector que continha o melhor num de inliers
			for(loop = 0; loop < parcial.size(); loop++) best.push_back(parcial[loop]); 
			for(loop = 0; loop < best.size(); loop++) inliers[loop] = best[loop];
			for(loop = 0; loop < 3; loop++) versor[loop] = (double) parcial_versor[loop];
			for(loop = 0; loop < 3; loop++) pb[loop] = (double) point_A[loop];
			parcial.clear();
		}
		else parcial.clear();
		
	}
	int num_inliers = best.size();
	return num_inliers;
}

int Ransac_2(double (*data)[3], double *versor, double* pb, int *inliers, double* charge, int number_it, double min_dist, int size, int mode){
	int indice1, indice2, loop, j;
	std::vector <int> parcial;
	double parcial_versor[3];
	std::vector <int> best;
	double distancia, norma1, norma2;
	int qtd = 0;
	double point_A[3];
	double point_B[3];
	double aux[3];
	double cross[3];
	double TCharge = 0;
	double PDF[size];
	double AvgCharge, TwiceAvCharge;
	double distancias_quadrado = 0, best_weight = 100000000000;

	if (mode == 2 || mode == 3){
		for (loop = 0; loop < size; loop++) TCharge += charge[loop];
		AvgCharge = (double) TCharge/size;
		TwiceAvCharge = (double) 2*AvgCharge;
		getPDF(charge, TCharge, size, PDF);
	}

	for(int i = 0; i <= number_it; i++){
		get_random(indice1, indice2, data, charge, PDF, TCharge, AvgCharge, TwiceAvCharge, mode, size);
		
		for(loop = 0; loop < 3; loop++) point_A[loop] = data[indice1][loop];
		
		for(loop = 0; loop < 3; loop++) point_B[loop] = data[indice2][loop];
		
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) point_B[loop] - point_A[loop];
		
		norma1 = (double)sqrt((double)pow(parcial_versor[0], 2) + (double)pow(parcial_versor[1], 2) + (double)pow(parcial_versor[2], 2));
		
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) parcial_versor[loop]/norma1;
		
		for(j = 0; j < size; j++){
			for(loop = 0; loop < 3; loop++) aux[loop] = point_A[loop] - data[j][loop];
			
			cross[0] = (double) parcial_versor[1]*aux[2] - parcial_versor[2]*aux[1];
			cross[1] = (double) -(parcial_versor[0]*aux[2] - parcial_versor[2]*aux[0]);
			cross[2] = (double) parcial_versor[0]*aux[1] - parcial_versor[1]*aux[0];
			
			norma2 = sqrt(pow(cross[0], 2.0) + pow(cross[1], 2.0) + pow(cross[2], 2.0));
			if(fabs(norma2) <= min_dist){
			parcial.push_back(j);
			distancias_quadrado += (double) pow(norma2, 2.0);	
			} 
		}
		if(best_weight > (double) distancias_quadrado/parcial.size()){ // parcial.size() > best.size()
			best.clear(); // Limpa o vector que continha o melhor num de inliers
			best_weight = (double) distancias_quadrado/parcial.size();
			for(loop = 0; loop < parcial.size(); loop++) best.push_back(parcial[loop]); 
			for(loop = 0; loop < best.size(); loop++) inliers[loop] = best[loop];
			for(loop = 0; loop < 3; loop++) versor[loop] = (double) parcial_versor[loop];
			for(loop = 0; loop < 3; loop++) pb[loop] = (double) point_A[loop];
			parcial.clear();
		}
		else parcial.clear();
		distancias_quadrado = 0.0;	
		
	}
	int num_inliers = best.size();
	return num_inliers;
}

void Fit3D(double *vX, double *vY, double *vZ, double *vQ, double *versor, double *Pb, int size){
	// 3D Line Regression
	// adapted from: https://github.com/jczamorac/Tracking_RANSAC/

	//long double PI = 3.14159265358979323851;
	
	int R, C;
	double Q;
	double Xm,Ym,Zm;
	double Xh,Yh,Zh;
	double a,b;
	double Sxx,Sxy,Syy,Sxz,Szz,Syz;
	double theta;
	double K11,K22,K12,K10,K01,K00;
	double c0,c1,c2;
	double p,q,r,dm2;
	double rho,phi;
	int i;

	Q=Xm=Ym=Zm=0.;
	double total_charge=0;
	Sxx=Syy=Szz=Sxy=Sxz=Syz=0.;

	for (i = 0; i < size; i++){
	Q+=vQ[i]/10.;
	Xm+=vX[i]*vQ[i]/10.;
	Ym+=vY[i]*vQ[i]/10.;
	Zm+=vZ[i]*vQ[i]/10.;
	Sxx+=vX[i]*vX[i]*vQ[i]/10.;
	Syy+=vY[i]*vY[i]*vQ[i]/10.;
	Szz+=vZ[i]*vZ[i]*vQ[i]/10.;
	Sxy+=vX[i]*vY[i]*vQ[i]/10.;
	Sxz+=vX[i]*vZ[i]*vQ[i]/10.;
	Syz+=vY[i]*vZ[i]*vQ[i]/10.;
	}

	Xm/=Q;
	Ym/=Q;
	Zm/=Q;
	Sxx/=Q;
	Syy/=Q;
	Szz/=Q;
	Sxy/=Q;
	Sxz/=Q;
	Syz/=Q;
	Sxx-=(Xm*Xm);
	Syy-=(Ym*Ym);
	Szz-=(Zm*Zm);
	Sxy-=(Xm*Ym);
	Sxz-=(Xm*Zm);
	Syz-=(Ym*Zm);

	theta=0.5*atan((2.*Sxy)/(Sxx-Syy));

	K11=(Syy+Szz)*pow(cos(theta),2)+(Sxx+Szz)*pow(sin(theta),2)-2.*Sxy*cos(theta)*sin(theta);
	K22=(Syy+Szz)*pow(sin(theta),2)+(Sxx+Szz)*pow(cos(theta),2)+2.*Sxy*cos(theta)*sin(theta);
	K12=-Sxy*(pow(cos(theta),2)-pow(sin(theta),2))+(Sxx-Syy)*cos(theta)*sin(theta);
	K10=Sxz*cos(theta)+Syz*sin(theta);
	K01=-Sxz*sin(theta)+Syz*cos(theta);
	K00=Sxx+Syy;

	c2=-K00-K11-K22;
	c1=K00*K11+K00*K22+K11*K22-K01*K01-K10*K10;
	c0=K01*K01*K11+K10*K10*K22-K00*K11*K22;


	p=c1-pow(c2,2)/3.;
	q=2.*pow(c2,3)/27.-c1*c2/3.+c0;
	r=pow(q/2.,2)+pow(p,3)/27.;


	if(r>0) dm2=-c2/3.+pow(-q/2.+sqrt(r),1./3.)+pow(-q/2.-sqrt(r),1./3.);
	if(r<0)
	{
	rho=sqrt(-pow(p,3)/27.);
	phi=acos(-q/(2.*rho));
	dm2=std::min(-c2/3.+2.*pow(rho,1./3.)*cos(phi/3.),std::min(-c2/3.+2.*pow(rho,1./3.)*cos((phi+2.*M_PI)/3.),-c2/3.+2.*pow(rho,1./3.)*cos((phi+4.*M_PI)/3.)));
	}

	a=-K10*cos(theta)/(K11-dm2)+K01*sin(theta)/(K22-dm2);
	b=-K10*sin(theta)/(K11-dm2)-K01*cos(theta)/(K22-dm2);

	Xh=((1.+b*b)*Xm-a*b*Ym+a*Zm)/(1.+a*a+b*b);
	Yh=((1.+a*a)*Ym-a*b*Xm+b*Zm)/(1.+a*a+b*b);
	Zh=((a*a+b*b)*Zm+a*Xm+b*Ym)/(1.+a*a+b*b);

	versor[0] = (double) Xh - Xm;
	versor[1] = (double) Yh - Ym;
	versor[2] = (double) Zh - Zm;

	double norma = sqrt( pow(versor[0], 2.) + pow(versor[1], 2.) + pow(versor[2], 2.) );

	for(i = 0; i < 3; i++) versor[i] = (double) versor[i]/norma;


	Pb[0] = Xm;
	Pb[1] = Ym;
	Pb[2] = Zm;
}

}


