#pragma once
#include "cdisccontroller.h"
#include "CParams.h"
#include "CDiscCollisionObject.h"
#include <cmath>
#include <vector>

typedef unsigned int uint;
class CQLearningController :
	public CDiscController
{
private:
	uint _grid_size_x;
	uint _grid_size_y;
	//Q-learning paramaters
	double discountRate = 0.8;
	double learningRate = 0.2;

	//Q-table
	//NOTE: one shared Q table to allow for faster learning and convergence(shared knowledge).
	std::vector<std::vector<std::vector<double> > > qSharedTbl; //right-up-left-down == 0-1-2-3
	
	//To allow for updating the Q-table once after the sweeper has died
	void theRessurection();
	std::vector<bool> killConfirmed;

public:
	CQLearningController(HWND hwndMain);
	virtual void InitializeLearningAlgorithm(void);
	double R(uint x, uint y, uint sweeper_no);
	virtual bool Update(void);
	virtual ~CQLearningController(void);
};

