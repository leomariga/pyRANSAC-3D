#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
#include <random>

extern "C" {

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

	if (mode == 2){
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
			if(norma2 <= min_dist)parcial.push_back(j); 
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
	double distancias_quadrado = 0, best_weight = 0;

	if (mode == 2){
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
			if(norma2 <= min_dist){
			parcial.push_back(j);
			distancias_quadrado += (double) pow(norma2, 2.0);	
			} 
		}
		if(best_weight < (double) distancias_quadrado/parcial.size()){ // parcial.size() > best.size()
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

}


