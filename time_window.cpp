// finalproject-with_shortage.cpp : ���ɮץ]�t 'main' �禡�C�{���|��ӳB�}�l����ε�������C
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <climits>
#include "gurobi_c++.h"
#include<algorithm>



using namespace std;
#define N 43

int main(int  argc, char argv[])
{

	fstream distance;
	distance.open("distance.txt", ios::in);
	double d[N][N] = { 0 };
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			distance >> d[i][j];
		}
	}

	fstream demand;
	demand.open("demand.txt", ios::in);
	int de[N] = { 0 };
	for (int i = 0; i < N; i++)
	{
		demand >> de[i];
	}

	fstream time;
	time.open("time.txt", ios::in);
	int t[N][N] = { 0 };
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			time >> t[i][j];
		}
	}

	fstream service_time;
	service_time.open("service_time.txt", ios::in);
	int s[N] = { 0 };
	for (int i = 0; i < N; i++)
	{
		service_time >> s[i];
	}


	int e[N] = { 0 };
	int num_shortage;
	int shortage_node;
	int shortage;
	cout << "The total number of the shortage nodes: ";
	cin >> num_shortage;
	cout << "The shortage nodes & The shortage quantity: ";
	for (int i = 0; i < num_shortage; i++)
	{
		cin >> shortage_node >> shortage;
		e[shortage_node - 1] = shortage;
	}

	/*fstream shortage;
	shortage.open("shortage.txt", ios::in);
	int e[N] = { 0 };
	for (int i = 0; i < N; i++)
	{
		shortage >> e[i];
	}*/

	int y[3] = { 6, 6, 3 };
	int y_ca[15] = { 250,250,250,250,250,250,500,500,500,500,500,500,750,750,750 };

	int l[3][N];                          //lower bound
	for(int p=0;p<3;p++){
		for (int i = 0; i < N; i++)
		{
			l[p][i] = 0;//�@�}�l(11�I)
		}
	}
	int u[3][N];                          //upper bound
	for (int p = 0; p < 3; p++) {
		for (int i = 0; i < N; i++)
		{
			u[p][i] = 300;//�@�}�l(11�I)
		}
	}

	try
	{
		GRBEnv env = GRBEnv();

		GRBModel model = GRBModel(env);

		GRBVar x[3][15][N][N];
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						x[p][v][i][j] = model.addVar(0, 1, 0, GRB_BINARY);
					}
				}
			}
		}
		//�ɶ����W�U��
		GRBVar ti[3][5][N];
		for (int p=0;p<3;p++){
		for (int v = 0; v < 5; v++)
		{
			for (int i = 0; i < N; i++)
			{
				ti[p][v][i] = model.addVar(l[p][i], u[p][i], 0, GRB_CONTINUOUS);
			}
		}
		}
		model.update();

		/* �]�w�ؼЦ��P��� */

		GRBLinExpr sum = 0;
		GRBLinExpr sum2 = 0;
		GRBLinExpr sum3 = 0;


		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						sum += d[i][j] * x[p][v][i][j];
					}
				}
			}
		}

		model.setObjective(sum, GRB_MINIMIZE);

		// Add constraint �U���u�u���@�x��
		//for (int p = 0; p < 3; p++)
		{
			for (int i = 1; i < N; i++)
			{
				for (int j = 0; j < N; j++)
				{
					if (i != j)
					{
						sum = 0;
						for (int p = 0; p < 3; p++)
						{
							for (int v = 0; v < 15; v++)
							{
								sum += x[p][v][i][j];
							}
						}
						model.addConstr(sum <= 1);
					}
				}
			}
		}

		// Add constraint 2 �����ݨD
		sum = 0;
		for (int i = 0; i < N; i++)
		{
			sum += de[i];
		}
		for (int v = 0; v < 15; v++)
		{
			sum2 += y_ca[v];
		}
		model.addConstr(sum <= sum2);

		//Add constraint �y�q�u��(�C�������n����)

		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 0; v < 6; v++)
			{
				for (int j = 2; j < N; j++)
				{
					sum += x[p][v][1][j];
				}
			}
			model.addConstr(sum == 2);
		}

		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 6; v < 12; v++)
			{
				for (int j = 2; j < N; j++)
				{
					sum += x[p][v][1][j];
				}
			}
			model.addConstr(sum == 2);
		}

		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 12; v < 15; v++)
			{
				for (int j = 2; j < N; j++)
				{
					sum += x[p][v][1][j];
				}
			}
			model.addConstr(sum == 1);
		}

		//Add constraint �y�q�u��(�C���I���n�g�L�A�B�u���@�x���|�g�L)
		for (int i = 2; i < N; i++)
		{
			sum = 0;
			for (int p = 0; p < 3; p++)
			{
				for (int v = 0; v < 15; v++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i != j)
							sum += x[p][v][i][j];
					}
				}
			}
			model.addConstr(sum == 1);
		}

		//Add constraint �y�q�u��(�U�ݨD�I�i=�X) ?!?!?!?!
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					sum = 0;
					sum2 = 0;
					for (int j = 0; j < N; j++)
					{
						for (int k = 0; k < N; k++)
						{
							if (i != j && j != k && k != i)
							{
								sum += x[p][v][i][j];
								sum2 += x[p][v][k][i];
							}
						}
					}
					model.addConstr(sum == sum2);
				}
			}
		}

		//���������qO�X�o��DC

		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 0; v < 6; v++)
			{
				sum += x[p][v][0][1];
			}
			model.addConstr(sum == 2);
		}

		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 6; v < 12; v++)
			{
				sum += x[p][v][0][1];
			}
			model.addConstr(sum == 2);
		}


		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 12; v < 15; v++)
			{
				sum += x[p][v][0][1];
			}
			model.addConstr(sum == 1);
		}


		//�������o��O�������X�ܻݨD�I
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int j = 2; j < N; j++)
				{
					sum += x[p][v][0][j];
				}
			}
		}
		model.addConstr(sum == 0);

		//�������o��DC�����^��O
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				sum += x[p][v][1][0];
			}
		}
		model.addConstr(sum == 0);

		//���������^��O
		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 0; v < 6; v++)
			{
				for (int i = 2; i < N; i++)
				{
					sum += x[p][v][i][0];
				}
			}
			model.addConstr(sum == 2);
		}
		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 6; v < 12; v++)
			{
				for (int i = 2; i < N; i++)
				{
					sum += x[p][v][i][0];
				}
			}
			model.addConstr(sum == 2);
		}
		for (int p = 0; p < 3; p++)
		{
			sum = 0;
			for (int v = 12; v < 15; v++)
			{
				for (int i = 2; i < N; i++)
				{
					sum += x[p][v][i][0];
				}
			}
			model.addConstr(sum == 1);
		}

		//����l�j��
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 11)
						{
							if (j == 29)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 24)
						{
							if (j == 40)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 4)
						{
							if (j == 39)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 6)
						{
							if (j == 25)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		sum = 0;
		for (int p = 0; p < 3; p++)
		{

			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 15)
						{
							if (j == 35)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 16)
						{
							if (j == 30)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 3)
						{
							if (j == 23)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 9)
						{
							if (j == 20)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 26)
						{
							if (j == 37)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 5)
						{
							if (j == 19)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);
		sum = 0;
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i == 21)
						{
							if (j == 27)
							{
								sum += x[p][v][1][i] + x[p][v][i][j] + x[p][v][j][0];
							}
						}
					}
				}
			}
		}
		model.addConstr(sum >= 2);

		//Add constraint ��������-1
		for (int i = 0; i < N; i++)
		{
			if (i == 13 || i == 32 || i == 35 || i == 37 || i == 38 || i == 40)
			{
				sum = 0;
				for (int j = 0; j < N; j++)
				{
					for (int p = 0; p < 3; p++)
					{
						for (int v = 6; v < 15; v++)
						{
							sum += x[p][v][i][j];
						}
					}
				}
				model.addConstr(sum == 0);
			}
		}

		//Add constraint ��������-2
		for (int i = 0; i < N; i++)
		{
			if (i == 3 || i == 6 || i == 9 || i == 19 || i == 21 || i == 23 || i == 26 || i == 27 || i == 33 || i == 34)
			{
				sum = 0;
				for (int j = 0; j < N; j++)
				{
					for (int p = 0; p < 3; p++)
					{
						for (int v = 12; v < 15; v++)
						{
							sum += x[p][v][i][j];
						}
					}
				}
				model.addConstr(sum == 0);
			}
		}


		//Add constraint 6 �U�����e�q����
		for (int v = 0; v < 15; v++)
		{
			sum = 0;
			sum2 = 0;
			for (int i = 0; i < N; i++)
			{
				for (int p = 0; p < 3; p++)
				{
					for (int j = 0; j < N; j++)
					{
						if (i != j)
						{
							sum = y_ca[v];
							sum2 += x[p][v][i][j] * de[i];
						}
					}
				}
			}
			model.addConstr(sum >= sum2);
		}
		//(11)�ɶ�����
		for (int p = 0; p < 3; p++) {
			for (int v = 0; v < 15; v++) {
				for (int i = 0; i < N; i++)
				{
					for (int j = 1; j < N; j++)
					{
						if (i != j)
						{

							model.addConstr(ti[v][i] + s[i] + t[i][j] - ti[v][j] <= (1 - x[p][v][i][j])*9999);
						}
					}
				}
			}
		}
		// Optimize model
		model.update();		//��smodel

		model.optimize();	//�}�l�D��


		//Print out the result
		ofstream myfile;
		myfile.open("example.txt");
		for (int p = 0; p < 3; p++)
		{
			for (int v = 0; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						myfile << x[p][v][i][j].get(GRB_DoubleAttr_X) << " ";
					}
				}
			}
		}
		myfile.close();


		cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;//�C�L�̨θѪ���
		cout << endl;

		for (int p = 0; p < 3; p++)
		{
			cout << "Day " << p + 1 << " :" << endl;

			cout << "���� 1 :" << endl;
			for (int v = 0; v < 6; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						{
							{
								if (x[p][v][i][j].get(GRB_DoubleAttr_X) == 1)
								{
									cout << i + 1 << "->" << j + 1 << endl;
								}
							}
						}
					}
				}
			}
			cout << "���� 2 :" << endl;
			for (int v = 6; v < 12; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						{
							{
								if (x[p][v][i][j].get(GRB_DoubleAttr_X) == 1)
								{
									cout << i + 1 << "->" << j + 1 << endl;
								}
							}
						}
					}
				}
			}
			cout << "���� 3 :" << endl;
			for (int v = 12; v < 15; v++)
			{
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < N; j++)
					{
						{
							{
								if (x[p][v][i][j].get(GRB_DoubleAttr_X) == 1)
								{
									cout << i + 1 << "->" << j + 1 << endl;
								}
							}
						}
					}
				}
			}
			cout << endl;
		}
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

	system("pause");


	return(0);
}


