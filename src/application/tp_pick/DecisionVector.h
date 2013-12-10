enum decisionEnum {ROLL, KEEP};

class DecisionVector {
public:	
	decisionEnum dice[5];

	DecisionVector();
	DecisionVector(decisionEnum a[5]);

	
};

DecisionVector::DecisionVector()
{
	
}

DecisionVector::DecisionVector(decisionEnum a[5])	
{
	for (int i = 0; i<5; i++){
		dice[i] = a[i];
	}
}
