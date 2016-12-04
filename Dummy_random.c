#include<stdio.h>

///////////////////////////////////////////////////////////////////////////
unsigned char dist_arr[48] = {255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255};
unsigned char path[48];
int angle = 90;
unsigned char parent[48];
unsigned char sorted[48] = {0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0};
unsigned char sequence[] = {4, 32 , 4 , 25 , 18 , 14 , 16, 0};
unsigned int u_set[48][4] = {{205 , 2419, 0, 0},                      //1
							 {118 , 304, 0, 0},						  //2
							 {217, 403, 2800, 2720},				  //3
							 {316, 502, 0, 0},						  //4
							 {415, 601, 0, 0},						  //5
							 {514, 700, 0, 0},						  //6
							 {613, 823,3016, 3120},					  //7
							 {712, 922, 0, 0},						  //8
							 {811, 1021, 0, 0},						  //9
							 {910, 1120, 0, 0},						  //10
							 {1009, 1219, 3312, 3416},				  //11
							 {1108, 1318, 0, 0},				      //12
							 {1207, 1417,0 ,0},						  //13
							 {1306, 1516, 0, 0},					  //14
							 {1405, 1615, 3608, 3712},				  //15
							 {1504, 1714, 0, 0},					  //16
							 {1603, 1813, 0, 0},					  //17
							 {1702, 1912, 0, 0},					  //18
							 {1801, 2011, 3904, 4008},				  //19
							 {1900, 2110, 0,0},					      //20
							 {2023,2209,0,0},
							 {2122,2208,0,0},
							 {2221,2407,0,0},
							 {2320,100,0,0},
							 {2316,2604,0,0},						  //25
							 {4300,2708,4120,2516},
							 {308,2620,2804,0},
							 {312,2900,2716,0},
							 {2812,3004,3200,4420},
							 {2916,704,0,0},						 //30
							 {708,3220,0,0},						 
							 {3300,3108,2912,4516},
							 {1100,3212,3420,0},
							 {1104,3516,3308,0},
							 {4612,3404,3620,3816},					//35
							 {3508,1520,0,0},						  
							 {1500,3812,0,0},
							 {3700,4708,3504,3916},
							 {3804,1916,4012,0},
							 {3900,1920,4108,0},					 //40
							 {4020,4804,4212,2608},
							 {2312,4100,0,0},
							 {2612,4404,0,0},
							 {2908,4316,0,0},
							 {3204,4620,0,0},						 //45
							 {3500,4508,0,0},
							 {3820,4812,0,0},
							 {4116,4700,0,0}};
unsigned int R_set[48][2] = {{1,25},				//1
							 {2,0},
							 {3,0},
							 {4,0},
							 {5,27},				//5
							 {6,0},
							 {7,0},
							 {8,0},
							 {9,28},
							 {10,0},				//10
							 {11,0},
							 {12,0},
							 {13,30},
							 {14,0},
							 {15,0},				//15
							 {16,0},
							 {17,31},
							 {18,0},
							 {19,0},
							 {20,0},				//20
							 {21,33},
							 {22,0},
							 {23,0},
							 {24,0},
							 {25,0},				//25
							 {25,26},
							 {25,26},
							 {26,27},
							 {26,27},
							 {27,0},				//30
							 {28,0},
							 {28,29},
							 {28,29},
							 {29,30},
							 {29,30},				//35
							 {30,0},
							 {31,0},
							 {31,32},
							 {31,32},
							 {32,33},				//40
							 {32,33},
							 {33,0},
							 {26,0},
							 {26,0},
							 {29,0},				//45
							 {29,0},
							 {32,0},
							 {32,0} };		
void reset_all()
{
	int i = 0;
	while(i < 48)
	{
		sorted[i] = 0;
		dist_arr[i] = 255;
		path[i] = 0;
		parent[i] = 0;
		i++;
	}
}
void check_angle(void)
{
	if(angle < 0)
	angle += 360;
}
void navigate()
{
	int i = 0;
	int j = 0;
	int h_angle = 0;
	while(path[i+1]!=0)
	{
		for(j = 0; j < 4; j++)
		{
			if(u_set[path[i]-1][j]/100 == path[i+1])
			{
				h_angle = 15*(u_set[path[i]-1][j]%100);
				break;
			}
		}
		if((h_angle - angle) > -16 && (h_angle - angle) < 16)
		{
			printf("Following Line\n");
			angle = h_angle;
			
			
		}
		else{
			printf("h_angle = %d, angle = %d\n", h_angle, angle);
			if(h_angle > angle)
			{
				printf("Left turning by %d\n", h_angle-angle);
				printf("Following Line\n");
				angle = h_angle;
			}
			else
			{
				printf("Right turning by %d\n", angle-h_angle);
				printf("Following Line\n");
				angle = h_angle;
			}
			
		
		}
		check_angle();
		i++;
	}
	printf("Beep! Beep!\n");
}
void reverse_path()
{
	int i = 0, j = 0, temp = 0;
		while(path[i]!=0)
			i++;
		for(j = 0; j< (i)/2 ; j++)
		{
			temp = path[j];
			path[j] = path[i-1-j];
			path[i-1-j] = temp;
		}
}	

int shortest_path(int source_node, int destination)
{
	//printf("Shortest path called\n");
	reset_all();
	int i = 0, min_dist = 255, temp = 0, j = 0, k = 0, flag = 0;
	unsigned char  temp_parent = 1, min_node = 1;
	parent[source_node-1] = 200;
	sorted[source_node - 1] = 1;
	dist_arr[source_node - 1] = 0;
	while(i < 48)
	{
		min_dist = 255;
		flag = 0;
		for(j = 0; j < 48; j++)
		{
			if(sorted[j] == 1)
			{
				for(k = 0; k < 4; k++)
				{
					//printf("i = %d , j = %d , k = %d\n",i,j,k);
					if(u_set[j][k] == 0)
						break;
				else if(sorted[(u_set[j][k]/100)-1] == 1)
						continue;
					else{
						temp = dist_arr[j]+1;
						if(temp < min_dist)
						{
							flag = 1;
							temp_parent = j+1;
							min_node = u_set[j][k]/100;
							min_dist = temp;
						}
					}
				}
			}
		}
		if(flag == 1)
		{
			parent[min_node-1] = temp_parent;
			dist_arr[min_node-1] = min_dist;
			sorted[min_node-1] = 1;
			if(R_set[min_node - 1][0] == destination || R_set[min_node - 1][1] == destination)
				break;
		}
		i++;
	}
	//printf("while loop exit last node = %d\n",min_node);
	i = min_node;
	j = 0;
	while((parent[i - 1] != 200))
	{
	
	path[j] = i;
	i = parent[i-1];
	j++;
	}
	path[j] = i;
	reverse_path();
	return min_node;
}



int main()
{
	int i = 0,j = 47, x = sequence[0];
	int count = 0;
	while(sequence[i+1] != 0)
	{
		j = 0;
		x = shortest_path(x,sequence[i+1]);
		while(j <= 47)
		{
			if(path[j] != 0)
				printf("%d  ",path[j]);
			else
				break;
			j++;
		}
		printf("\n");
		navigate();
		
		i++;
		count++;
		if(count > 50)
		{
			printf("Counter Overflow!\n");
			break;
		}
		printf("**************************\n");
		
	}
	printf("Beeeeeeeeeeeppppppp!!!\n");
	return 0;
}