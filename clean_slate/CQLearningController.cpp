/**
         (                                      
   (     )\ )                                   
 ( )\   (()/(   (    ) (        (        (  (   
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(  
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\  
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_) 
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |  
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |  
                                        |___/   

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"
#include <vector>

CQLearningController::CQLearningController(HWND hwndMain):
	CDiscController(hwndMain),
	_grid_size_x(CParams::WindowWidth / CParams::iGridCellDim + 1),
	_grid_size_y(CParams::WindowHeight / CParams::iGridCellDim + 1)
{
}
/**
 The update method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm(void)
{
	//Initialize zero table
	for (uint i=0; i < _grid_size_x; i++) 
	{
		std::vector<std::vector<double>> yState;
		for (uint j=0; j < _grid_size_y; j++) 
		{
			yState.push_back({ 0,0,0,0 });
		}
		qSharedTbl.push_back(yState);
	}
	theRessurection();
}
/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no)
{
	double collision = ((*(m_vecSweepers[sweeper_no])).CheckForObject(m_vecObjects, CParams::dMineScale));
	int reward = 0;

	if (collision >= 0)
	{
		switch ((*m_vecObjects[collision]).getType())
		{
			case CDiscCollisionObject::Rock:
			{
				reward = -5;
				break;
			}
			case CDiscCollisionObject::Mine:
			{
				if (!(*m_vecObjects[collision]).isDead()) { reward = 50; }
				//else { reward = 1; }
				break;
			}
			case CDiscCollisionObject::SuperMine:
			{
				reward = -50;
				break;
			}
		}
	}
	//else { reward = -1; }
	return reward;
}
//Get higest policy value
double getExpectedReward(std::vector<double> actions)
{
	double eReward = actions[0];
	for (uint i = 1; i < 4; i++) { if (actions[i] > eReward) { eReward = actions[i]; } }
	return eReward;
}
//Initializes the killConfirmed boolean vector. To ensure qSharedTbl updates correctly
void CQLearningController::theRessurection()
{
	killConfirmed.clear();
	for (int i = 0; i < m_NumSweepers; i++) { killConfirmed.push_back(false);}
}

//Choose policy
int getPolicy(std::vector<double> actions)
{
	//Vec of all max policies
	std::vector<int> policies;

	double expR = getExpectedReward(actions);

	//Find and add all max policies to vec
	for (uint i = 0; i < 4; i++) { if (actions[i] == expR) { policies.push_back(i); } }

	//If random int is negative, choose a random policy
	/*int random = RandInt(-1,9);
	if (random < 0) 
	{
		return RandInt(0, actions.size() - 1);
	}*/
	
	//Choose randomly from optimal policies
	return policies[RandInt(0, policies.size() - 1)];
}
/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
bool CQLearningController::Update(void)
{
		//m_vecSweepers is the array of minesweepers
		//everything you need will be m_[something] ;)
		uint cDead = std::count_if(m_vecSweepers.begin(),
			m_vecSweepers.end(),
			[](CDiscMinesweeper * s)->bool {
			return s->isDead();
		});
		if (cDead == CParams::iNumSweepers) {
			printf("All dead ... skipping to next iteration\n");
			m_iTicks = CParams::iNumTicks;
			theRessurection();
		}

		for(uint sw = 0; sw < CParams::iNumSweepers; ++sw) 
		{
			if (m_vecSweepers[sw]->isDead()) continue;
			/**
			Q-learning algorithm according to:
			Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
			*/
			//1:::Observe the current state:
			SVector2D<int> currState = (*m_vecSweepers[sw]).Position();
			currState /= 10;
			//2:::Select action with highest historic return:
			int pol = getPolicy(qSharedTbl[currState.x][currState.y]);
			(*m_vecSweepers[sw]).setRotation((ROTATION_DIRECTION)pol);
		}

		CDiscController::Update(); //call the parent's class update. Do not delete this.

		for (uint sw = 0; sw < CParams::iNumSweepers; ++sw) 
		{
			if (killConfirmed[sw]== true && m_vecSweepers[sw]->isDead()) continue;
			else if (m_vecSweepers[sw]->isDead()) { killConfirmed[sw] = true; }
			//TODO:compute your indexes.. it may also be necessary to keep track of the previous state
			//3:::Observe new state:
			SVector2D<int> currState = ((*m_vecSweepers[sw]).Position());
			currState /= 10;
			SVector2D<int> prevState = (*m_vecSweepers[sw]).PrevPosition();
			prevState /= 10;

			int act = (int)m_vecSweepers[sw]->getRotation();

			//4:::Update _Q_s_a accordingly:
			qSharedTbl[prevState.x][prevState.y][act] += (learningRate * (R(currState.x, currState.y, sw) + (discountRate * getExpectedReward(qSharedTbl[currState.x][currState.y])) - qSharedTbl[prevState.x][prevState.y][act]));

		}
		if (m_iTicks == CParams::iNumTicks) {theRessurection();}
		return true;
	}

CQLearningController::~CQLearningController(void)
{
	//TODO: dealloc stuff here if you need to	
}






