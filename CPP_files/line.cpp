#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>


extern "C" {

int Ransac(double (*data)[3], int number_it, double min_dist, double *versor, int *inliers, int size){
	int indice1, indice2, loop, j;
	std::vector <int> parcial;
	double parcial_versor[3];
	std::vector <int> best;
	//int best[size];
	//int parcial[size];
	double distancia, norma1, norma2;
	int qtd = 0;
	double point_A[3];
	double point_B[3];
	double aux[3];
	double cross[3];
		
	for(int i = 0; i <= number_it; i++){
		//srand (time());
		indice1 = rand() % size;
		indice2 = rand() % size;
		//std::cout<< indice1 << " " << indice2<< "\n\n";
		while(indice1 == indice2) indice2 = rand() % size;
		
		for(loop = 0; loop < 3; loop++) point_A[loop] = data[indice1][loop];
		
		for(loop = 0; loop < 3; loop++) point_B[loop] = data[indice2][loop];
		
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) point_B[loop] - point_A[loop];
		
		norma1 = (double)sqrt((double)pow(parcial_versor[0], 2) + (double)pow(parcial_versor[1], 2) + (double)pow(parcial_versor[2], 2));
		
		//std::cout << "Versor1 : " << parcial_versor[0] << " " << parcial_versor[1] << " " << parcial_versor[2] <<"\ncom norma = " << norma;
		//std::cout<< norma << " \n";
		
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) parcial_versor[loop]/norma1;
				
		//std::cout << "\nVersor2 : " << parcial_versor[0] << " " << parcial_versor[1] << " " << parcial_versor[2] << "\n\n";
		
		for(j = 0; j < size; j++){
			for(loop = 0; loop < 3; loop++) aux[loop] = point_A[loop] - data[j][loop];
			
			cross[0] = (double) parcial_versor[1]*aux[2] - parcial_versor[2]*aux[1];
			cross[1] = (double) -(parcial_versor[0]*aux[2] - parcial_versor[2]*aux[0]);
			cross[2] = (double) parcial_versor[0]*aux[1] - parcial_versor[1]*aux[0];
			
			norma2 = sqrt(pow(cross[0], 2) + pow(cross[1], 2) + pow(cross[2], 2));
			//std::cout << parcial.size();
			if(norma2 <= min_dist) parcial.push_back(j);
			}
		if(parcial.size() > best.size()){
			best.clear(); // Limpa o vector que continha o melhor num de inliers
			for(loop = 0; loop < parcial.size(); loop++) best.push_back(parcial[loop]); 
			for(loop = 0; loop < best.size(); loop++) inliers[loop] = best[loop];
			for(loop = 0; loop < 3; loop++) versor[loop] = (double) parcial_versor[loop];
			parcial.clear();
			}
		else parcial.clear();	
		
	}
	int num_inliers = best.size();
	//std::cout << parcial.size();	
	//std::cout << best[0] << " " << best[1] << "\n\n";	
	//std::cout << "Inliers C++ = " << best.size() << "\n";
	//std::cout << "Versor c++: ";
	//std::cout << versor[0] << " " << versor[1] << " " << versor[2] << "\n";
	return num_inliers;
	}
}


