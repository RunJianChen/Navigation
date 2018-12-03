//
//  RRT(origin).hpp
//  RRT
//
//  Created by Chen~_~RJ on 2018/11/29.
//  Copyright © 2018 Chen~_~RJ. All rights reserved.
//

#ifndef RRT_origin__hpp
#define RRT_origin__hpp

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <math.h>
#include <cstdlib>

#define max_point_num 1000000

using namespace std;


class map_point
{
public:
    int x,y;
    
    map_point(int xx=0,int yy=0):x(xx),y(yy) {}
};


class RRT
{
private:
    int map_H,map_W,map_size,max_point,buffer_length,max_tracing_back;//define size of the map
    unsigned char * map;//map condition,0--obstacle,1--movable
    map_point nav_start,nav_destination,*vertices,*edges;
    double probably;
public:
    RRT(int Height,int Width,unsigned char *map_data,int start_H,int start_W,int des_H,int des_W): map_H(Height),map_W(Width),nav_start(start_H,start_W),nav_destination(des_H,des_W)
    {
        map_size = Height*Width;
        map = map_data;
        max_point = max_point_num;
        vertices = new map_point[max_point_num];
        edges = new map_point[max_point_num];
        probably = 0.03;
        max_tracing_back = 500;
        buffer_length = 1;
        srand((unsigned)time(0));
    }
    void Path_Generator();
    int findQNear(map_point q_rand, map_point* vertices, int length);
    double my_rand(int i, int j);
    map_point findQNew(map_point q_near, map_point q_rand, int delta_q);
    int isQGoalOnQNearQNewEdge(map_point q_near, map_point q_new, map_point q_goal);
    int isequal(map_point q1, map_point q2);
    int* fillSolutionPath(map_point* edges, map_point* vertices);
    int isEdgeBelongs(map_point q_near, map_point q_new, int num);
    double pdist2(map_point a, map_point b);
    void smooth(int* path, int delta, int len);
};



#endif /* RRT_origin__hpp */
