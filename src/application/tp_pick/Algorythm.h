#include <stdlib.h>
#include <vector>
#include <iostream>
//#include <conio.h>

using namespace std;

struct dicesVector {
	int dice[5];
};



class Algorythm {
private:
	dicesVector dices;
public:
	void load(vector<int>);
	void load(dicesVector);
	DecisionVector calculate();
};

