//
//  Qlearn.h
//  Qlearn
//
//  Created by Chen~_~RJ on 2018/11/10.
//  Copyright © 2018 Chen~_~RJ. All rights reserved.
//

#ifndef Qlearn_h
#define Qlearn_h

#include <iostream>
#include <stdlib.h>
#include <time.h>

#define Max_Cost 100000;
#define Converge_Ceiling 2000000;
#define Converge_Delta 0.3;

class map_point
{
private:
    int Height,Width;
public:
    map_point(int h=0,int w=0):Height(h),Width(w) {}
    void set_point(int h,int w)
    {
        Height = h;
        Width = w;
    }
    int Get_Height()
    {
        return Height;
    }
    int Get_Width()
    {
        return Width;
    }
};

class Factor_to_all_direction
{
public:
    int up,down,left,right;
    Factor_to_all_direction(int u=20,int d=20,int l=20,int r=20):up(u),down(d),left(l),right(r) {}
};

class Qlearn
{
private :
    int map_H,map_W,map_size,* random;//define size of the map
    unsigned char * map;//map condition,0--obstacle,1--movable
    Factor_to_all_direction * Qtable,* Reward;//define Qtable and Reward matrix
    int max_iteration,max_dis;//define the ceiling of iteration time,default 10000
    double learning_rate;//define learning rate
    map_point nav_start,nav_destination;
    
public:
    Qlearn(int Height,int Width,unsigned char *map_data,int start_H,int start_W,int des_H,int des_W,int iteration_limit=100000000,double rate=0.25): map_H(Height),map_W(Width),max_iteration(iteration_limit),learning_rate(rate),nav_start(start_H,start_W),nav_destination(des_H,des_W)
    {
        map_size = Height*Width;
        map = map_data;
        max_dis = Height*Height + Width*Width;
        Qtable = new Factor_to_all_direction[map_size];
        Reward = new Factor_to_all_direction[map_size];
        random = new int[map_size]{0};
    }
    int Get_point_ID(map_point *query);
    void Set_Reward();//set reward table
    void Qlearning();//main function for Qlearning
    bool update_Qtable(map_point *state,int action);//calculate transmission function
    void Path_Generator();
    int Get_Dis_to_Des(map_point* current);
    int Random_Dir(map_point *current);
    ~Qlearn()
    {
        delete map;
        delete Qtable;
        delete Reward;
    }
};
#endif /* Qlearn_h */
