#include "Algorythm.h"

void Algorythm::load(vector<int> in)
{
	if (in.size() < 6){
		for (unsigned int i=0; i<6; i++){
			dices.dice[i] = 0;
		}
		for (unsigned int i=0; i<in.size(); i++){
			dices.dice[i] = in[i];
		}
	}
}

DecisionVector Algorythm::calculate()
{
	//wektor przechowuje liczbe wyst¹pieñ oczek 0,1,2,3,4,5,6
	vector<int> count (7,0);
	
	//wektor przechowuje liczebnoœæ grup 0,1,2,3,4,5
	vector<int> twins (6,0);

	for (unsigned int i=0; i<6; i++){
		if (dices.dice[i] !=0) count[dices.dice[i]]++;
	}

	cout << "Zliczanie: " << endl;
	for (unsigned int i=0; i<count.size(); i++){
		
		cout << " " << count[i];
	}
	cout << endl;

	for(unsigned int i=0; i<7; i++){
		twins[count[i]]++;
	}

	cout << "Grupowanie: " << endl;
	for (unsigned int i=0; i<twins.size(); i++){
		
		cout << " " << twins[i];
	}
	cout << endl;
	
	
	//wszystkie jednakowe
	if (twins[5] == 1){
		cout << "GENERAL" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		return DecisionVector (situation);
	}
	
	//czwórka i dowolna
	if (twins[4] == 1){
		cout << "CZWÓRKA" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		for (unsigned int i=1; i<7; i++){
			if (count[i]==1) {
				for (unsigned int j=0; j<5; j++){
					if (dices.dice[j] == i){
						situation[j] = ROLL;
						//cout << "ROLL: " << j << endl;
					}
				}
			}
		}
		return DecisionVector (situation);
	}

	//trójka i dwójka
	if (twins[3] == 1 && twins[2] == 1){
		cout << "TRÓJKA I DWÓJKA" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		return DecisionVector (situation);
	}

	//trójka i dwie dowolne
	if (twins[3] == 1 && twins[2] != 1){
		cout << "TRÓJKA" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		for (unsigned int i=1; i<7; i++){
			if (count[i]==1) {
				for (unsigned int j=0; j<5; j++){
					if (dices.dice[j] == i){
						situation[j] = ROLL;
						//cout << "ROLL: " << j << endl;
					}
				}
			}
		}
		return DecisionVector (situation);
	}

	//dwie dwójki i dowolna
	if (twins[2] == 2){
		cout << "DWIE DWÓJKI" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		for (unsigned int i=1; i<7; i++){
			if (count[i]==1) {
				for (unsigned int j=0; j<5; j++){
					if (dices.dice[j] == i){
						situation[j] = ROLL;
						//cout << "ROLL: " << j << endl;
					}
				}
			}
		}
		return DecisionVector (situation);
	}

	//dwójka i trzy dowolne
	if (twins[2] == 1 && twins[3] != 1){
		cout << "DWÓJKA" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		for (unsigned int i=1; i<7; i++){
			if (count[i]==1) {
				for (unsigned int j=0; j<5; j++){
					if (dices.dice[j] == i){
						situation[j] = ROLL;
						//cout << "ROLL: " << j << endl;
					}
				}
			}
		}
		return DecisionVector (situation);
	}

	//ka¿da inna
	if (twins[2] == 0 && twins[3] == 0 && twins[4] == 0 && twins[5] == 0){
		cout << "NIC" << endl;
		decisionEnum situation[] = {KEEP, KEEP, KEEP, KEEP, KEEP};
		int miss = 1;
		for (int i=1; i<7; ++i){
			if (count[i] == 0) miss = i;
		}
		cout << "MISS:" << miss << endl;
		if (miss == 1 || miss == 6) return DecisionVector (situation);
		for (unsigned int i=0; i<5; i++){
			if (dices.dice[i] == 1)	situation[i] = ROLL;
		}
		return DecisionVector (situation);
	}
		


	//niezidentyfikowana
	cout << "UNRECOGNIZED" << endl;
	decisionEnum situation[] = {ROLL, ROLL, ROLL, ROLL, ROLL};
	DecisionVector decision = DecisionVector(situation);
	return decision;
}
