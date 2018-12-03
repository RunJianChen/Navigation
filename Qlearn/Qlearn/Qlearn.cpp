//
//  Qlearn.cpp
//  Qlearn
//
//  Created by Chen~_~RJ on 2018/11/10.
//  Copyright © 2018 Chen~_~RJ. All rights reserved.
//

#include "Qlearn.h"

void Qlearn::Qlearning()
{
    int count=0,converge=0,con_threhold = Converge_Ceiling;
    map_point *current = new map_point(nav_start);
    
    Set_Reward();
    
    while (count<max_iteration)
    {
        //randomly choose a direction
        int dir = Random_Dir(current);
        
        //update Qtable
        if (update_Qtable(current, dir))
            converge++;
        else
            converge = 0;
        
        //if converged finish
        if (converge > con_threhold)
            break;
        
        count++;
    }
    //std::cout << count << " " << " " << converge;
}

//given height and width,return the ID of the point
int Qlearn::Get_point_ID(map_point *query)
{
    int x = query->Get_Height(),y = query->Get_Width();
    if ((x>=0) && (x<map_H) && (y>=0) && (y<map_W))
        return (x*map_W+y);
    else
        return -1;
}

//Set reward matrix
void Qlearn::Set_Reward()
{
    int x,y;
    for (x = 0; x<map_H ; x++)
    {
        for (y = 0; y<map_W ; y++)
        {
            int now =  static_cast<int>(map[Get_point_ID(new map_point(x,y))]);
            if (Get_point_ID(new map_point(x-1,y))>=0)
                Reward[Get_point_ID(new map_point(x,y))].up += now - Get_Dis_to_Des((new map_point(x-1,y))) - static_cast<int>(map[Get_point_ID(new map_point(x-1,y))]);
            else Reward[Get_point_ID(new map_point(x,y))].up += now - Max_Cost;
            
            if (Get_point_ID(new map_point(x+1,y))>=0)
                Reward[Get_point_ID(new map_point(x,y))].down += now - Get_Dis_to_Des((new map_point(x+1,y))) - static_cast<int>(map[Get_point_ID(new map_point(x+1,y))]);
            else Reward[Get_point_ID(new map_point(x,y))].down += now - Max_Cost;
            
            if (Get_point_ID(new map_point(x,y-1))>=0)
                Reward[Get_point_ID(new map_point(x,y))].left += now - Get_Dis_to_Des((new map_point(x,y-1))) - static_cast<int>(map[Get_point_ID(new map_point(x,y-1))]);
            else Reward[Get_point_ID(new map_point(x,y))].left += now - Max_Cost;
            
            if (Get_point_ID(new map_point(x,y+1))>=0)
                Reward[Get_point_ID(new map_point(x,y))].right +=  now - Get_Dis_to_Des((new map_point(x,y+1))) - static_cast<int>(map[Get_point_ID(new map_point(x,y+1))]);
            else Reward[Get_point_ID(new map_point(x,y))].right += now - Max_Cost;
        }
    }
}

bool Qlearn::update_Qtable(map_point *state,int action)
{
    int next_max,delta=0,converge_delta = Converge_Delta;
    
    if (action == 0)
    {
        map_point *next_state = new map_point(state->Get_Height()-1,state->Get_Width());
        if (Get_point_ID(next_state)>=0)
        {
            next_max = std::max(Qtable[Get_point_ID(next_state)].up,Qtable[Get_point_ID(next_state)].down);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].left);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].right);
            delta = Qtable[Get_point_ID(state)].up;
            Qtable[Get_point_ID(state)].up = Reward[Get_point_ID(state)].up + learning_rate * next_max;
            delta -= Qtable[Get_point_ID(state)].up;
            state->set_point(state->Get_Height()-1,state->Get_Width());
        }
        else
            Qtable[Get_point_ID(state)].up = -Max_Cost;
        
    }
    else if (action == 1)
    {
        map_point *next_state = new map_point(state->Get_Height()+1,state->Get_Width());
        if (Get_point_ID(next_state)>=0)
        {
            next_max = std::max(Qtable[Get_point_ID(next_state)].up,Qtable[Get_point_ID(next_state)].down);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].left);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].right);
            delta = Qtable[Get_point_ID(state)].down;
            Qtable[Get_point_ID(state)].down = Reward[Get_point_ID(state)].down + learning_rate * next_max;
            delta -= Qtable[Get_point_ID(state)].down;
            state->set_point(state->Get_Height()+1,state->Get_Width());
        }
        else
            Qtable[Get_point_ID(state)].down = -Max_Cost;
       
    }
    else if (action == 2)
    {
        map_point *next_state = new map_point(state->Get_Height(),state->Get_Width()-1);
        if (Get_point_ID(next_state)>=0)
        {
            next_max = std::max(Qtable[Get_point_ID(next_state)].up,Qtable[Get_point_ID(next_state)].down);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].left);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].right);
            delta = Qtable[Get_point_ID(state)].left;
            Qtable[Get_point_ID(state)].left = Reward[Get_point_ID(state)].left + learning_rate * next_max;
            delta -= Qtable[Get_point_ID(state)].left;
            state->set_point(state->Get_Height(),state->Get_Width()-1);
        }
        else
            Qtable[Get_point_ID(state)].left = -Max_Cost;
        
    }
    else if (action == 3)
    {
        map_point *next_state = new map_point(state->Get_Height(),state->Get_Width()+1);
        if (Get_point_ID(next_state)>=0)
        {
            next_max = std::max(Qtable[Get_point_ID(next_state)].up,Qtable[Get_point_ID(next_state)].down);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].left);
            next_max = std::max(next_max,Qtable[Get_point_ID(next_state)].right);
            delta = Qtable[Get_point_ID(state)].right;
            Qtable[Get_point_ID(state)].right = Reward[Get_point_ID(state)].right + learning_rate * next_max;
            delta -= Qtable[Get_point_ID(state)].right;
            state->set_point(state->Get_Height(),state->Get_Width()+1);
        }
        else
            Qtable[Get_point_ID(state)].right = -Max_Cost;
        
    }
    if (std::abs(delta)<= converge_delta)
        return true;
    else
        return false;
}
    
void Qlearn::Path_Generator()
{
    Qlearning();
    
    map_point* current=new map_point(nav_start);
    
    int *state = new int[map_size];
    
    for (int i=0;i<map_size;i++) state[i]=0;
    state[Get_point_ID(current)]=1;
   
    while ((current->Get_Height()!=nav_destination.Get_Height()) || (current->Get_Width()!=nav_destination.Get_Width()))
    {
        std::cout << current->Get_Height() << " " << current->Get_Width() << std::endl;
        int dir = -1 , max_reward = -100;
        if ((max_reward<Qtable[Get_point_ID(current)].up) && (!state[Get_point_ID(new map_point(current->Get_Height()-1,current->Get_Width()))]))
        {
            max_reward = Qtable[Get_point_ID(current)].up;
            dir = 0;
            state[Get_point_ID(new map_point(current->Get_Height()-1,current->Get_Width()))]=1;
        }
        if ((max_reward<Qtable[Get_point_ID(current)].down) && (!state[Get_point_ID(new map_point(current->Get_Height()+1,current->Get_Width()))]))
        {
            max_reward = Qtable[Get_point_ID(current)].down;
            dir = 1;
            state[Get_point_ID(new map_point(current->Get_Height()+1,current->Get_Width()))]=1;
        }
        if ((max_reward<Qtable[Get_point_ID(current)].left) && (!state[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()-1))]))
        {
            max_reward = Qtable[Get_point_ID(current)].left;
            dir = 2;
            state[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()-1))]=1;
        }
        if ((max_reward<Qtable[Get_point_ID(current)].right) && (!state[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()+1))]))
        {
            max_reward = Qtable[Get_point_ID(current)].right;
            dir = 3;
            state[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()+1))]=1;
        }
        switch (dir) {
            case 0:
                current->set_point(current->Get_Height()-1,current->Get_Width());
                break;
            case 1:
                current->set_point(current->Get_Height()+1,current->Get_Width());
                break;
            case 2:
                current->set_point(current->Get_Height(),current->Get_Width()-1);
                break;
            case 3:
                current->set_point(current->Get_Height(),current->Get_Width()+1);
                break;
                
            default:
                break;
        }
        
        
    }
    
   
    for (int i=0;i<map_H;i++)
        for (int j=0;j<map_W;j++)
        {
            //std::cout << Reward[i*map_W+j].up << " " << Reward[i*map_W+j].down << " "<<Reward[i*map_W+j].left<<" "<<Reward[i*map_W+j].right<<std::endl;
          std::cout << Qtable[i*map_W+j].up << " " << Qtable[i*map_W+j].down << " "<<Qtable[i*map_W+j].left<<" "<<Qtable[i*map_W+j].right<<std::endl;
        }
    
}

int Qlearn::Random_Dir(map_point *current)
{
    srand((unsigned)time(NULL));
    int direction = rand() % 4,count(0);
    
    int u(0),d(0),l(0),r(0),max_query,query_now;
    
    if (Get_point_ID(new map_point(current->Get_Height()-1,current->Get_Width()))>=0)
    u = random[Get_point_ID(new map_point(current->Get_Height()-1,current->Get_Width()))];
    
    if (Get_point_ID(new map_point(current->Get_Height()+1,current->Get_Width()))>=0)
    d = random[Get_point_ID(new map_point(current->Get_Height()+1,current->Get_Width()))];
    
    if (Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()-1))>=0)
    l = random[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()-1))];
    
    if (Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()+1))>=0)
    r = random[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()+1))];
    
    max_query = std::max(u,d);
    max_query = std::max(max_query,l);
    max_query = std::max(max_query,r);
    query_now = max_query;
    while ((query_now >= max_query) && ((u!=d) || (u!=l) || (u!=r) || (d!=l) || (d!=r) || (l!=r)) && (count<=30))
    {
        
        direction ++;
        direction = direction % 4;
        switch (direction)
        {
            case 0:
                if (Get_point_ID(new map_point(current->Get_Height()-1,current->Get_Width()))>=0)
                    query_now = u;
                break;
            case 1:
                if (Get_point_ID(new map_point(current->Get_Height()+1,current->Get_Width()))>=0)
                    query_now = d;
                break;
            case 2:
                if (Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()-1))>=0)
                    query_now = l;
                break;
            case 3:
                if (Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()+1))>=0)
                    query_now = r;
                break;
            
            default:
                break;
        }
        count ++;
    }
    switch (direction)
    {
        case 0:
            random[Get_point_ID(new map_point(current->Get_Height()-1,current->Get_Width()))]++;
            break;
        case 1:
            random[Get_point_ID(new map_point(current->Get_Height()+1,current->Get_Width()))]++;
            break;
        case 2:
            random[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()-1))]++;
            break;
        case 3:
            random[Get_point_ID(new map_point(current->Get_Height(),current->Get_Width()+1))]++;
            break;
            
        default:
            break;
    }
    return direction;
}

int Qlearn::Get_Dis_to_Des(map_point *current)
{
    int dis;
    dis = (current->Get_Height()-nav_destination.Get_Height())*(current->Get_Height()-nav_destination.Get_Height()) + (current->Get_Width()-nav_destination.Get_Width())*(current->Get_Width()-nav_destination.Get_Width());
    return dis*100/(10*10+10*10);
}
