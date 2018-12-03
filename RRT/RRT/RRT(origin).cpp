//
//  RRT(origin).cpp
//  RRT
//
//  Created by Chen~_~RJ on 2018/11/29.
//  Copyright © 2018 Chen~_~RJ. All rights reserved.
//

#include "RRT(origin).hpp"

void RRT::Path_Generator()
{
    int i = 0;
    int path_len = 0;
    int delta = 5;
    map_point q_rand;
    map_point q_near;
    map_point q_new;
    int *Path;
    vertices[0] = nav_start;
    
    for (i = 0; i < max_point; i++)
    {
        if (my_rand(0, 1) < probably)
            q_rand = nav_destination;
        else
        {
            q_rand.x = my_rand(1, map_W);
            q_rand.y = my_rand(1, map_H);
        }
        
        int qNearIndex = findQNear(q_rand, vertices, buffer_length);
        q_near = vertices[qNearIndex];
        q_new = findQNew(q_near, q_rand, max_tracing_back);
        
        if(q_new.x < 1 || q_new.y < 1 || q_new.x > map_W || q_new.y > map_H)
            continue;
        if (static_cast<int>(map[q_new.y*map_W+q_new.x]) == 0)
        {
            if (isEdgeBelongs(q_near, q_new, 1))
            {
                vertices[buffer_length] = q_new;
                edges[buffer_length - 1].x = buffer_length+1;
                edges[buffer_length - 1].y = qNearIndex+1;
                
                cout << "vertices = " << vertices[buffer_length-1].x << " " << vertices[buffer_length-1].y << endl;
                cout << "edges = " << edges[buffer_length-1].x << " " << edges[buffer_length-1].y << endl;
                
                if (isequal(q_new, nav_destination) || isQGoalOnQNearQNewEdge(q_near, q_new, nav_destination))
                {
                    
                    if (!isequal(q_new, nav_destination))
                    {
                        vertices[buffer_length] = nav_destination;
                        
                    }
                    
                    cout << "vertices = " << vertices[buffer_length].x << " " << vertices[buffer_length].y << endl;
                    cout << "helloworld" << endl;
                    Path = fillSolutionPath(edges, vertices);
                    while (*Path!=0)
                    {
                        cout << "   " << *Path ;
                        Path++;
                        path_len++;
                    }
                    int i0 = path_len;
                    while (i0)
                    {
                        Path--;
                        i0--;
                    }
                                        smooth(Path, delta, path_len);
                    return;
                }
                buffer_length++;
            }
        }
    }
    
}

int RRT::findQNear(map_point q_rand, map_point* vertices, int length)
{
    int i;
    double distance[max_tracing_back];
    for (i = 0;i < length;i++)
        distance[i] = pdist2(q_rand, vertices[i]);
    int minDistanceIndex = 0;
    for (i = 0; i < length; i++)
    {
        if (distance[i] < distance[minDistanceIndex])
            minDistanceIndex = i;
    }
    return minDistanceIndex;
}

double RRT::my_rand(int i, int j)
{
    double number = (rand() % 1000 / 1000.0)*(j - i) + i;
    return number;
}

map_point RRT::findQNew(map_point q_near, map_point q_rand, int delta_q)
{
    map_point v;
    v.x = q_rand.x - q_near.x;
    v.y = q_rand.y - q_near.y;
    double u[2];
    u[0] = v.x / sqrt(v.x*v.x+ v.y*v.y);
    u[1] = v.y / sqrt(v.x*v.x + v.y*v.y);
    
    map_point q_new;
    
    q_new.x = q_near.x + int(delta_q * u[0]);
    q_new.y = q_near.y + int(delta_q * u[1]);
    
    return q_new;
}

int RRT::isQGoalOnQNearQNewEdge(map_point q_near, map_point q_new, map_point q_goal)
{
    int isQGoalOnEdge;
    map_point v;
    map_point q_goal_hat;
    v.x = q_new.x - q_near.x;
    v.y = q_new.y - q_near.y;
    double distance = sqrt(v.x*v.x + v.y*v.y);
    double u[2];
    u[0] = v.x / distance;
    u[1] = v.y / distance;
    
    v.x = q_goal.x - q_near.x;
    v.y = q_goal.y - q_near.y;
    double distanceQNearQGoal = sqrt(v.x*v.x + v.y*v.y);;
    if (distanceQNearQGoal > distance)
    {
        return 0;
    }
    q_goal_hat.x = q_near.x + distanceQNearQGoal * u[0];
    q_goal_hat.y = q_near.y + distanceQNearQGoal * u[1];
    isQGoalOnEdge = isequal(q_goal_hat, q_goal);
    return isQGoalOnEdge;
}

int RRT::isequal(map_point q1, map_point q2)
{
    if (q1.x == q2.x&&q1.y == q2.y)
    {
        return 1;
    }
    else
        return 0;
}

int* RRT::fillSolutionPath(map_point* edges, map_point* vertices)
{
    int *path = new int[1000];
    
    for (int i = 0 ; i < 1000 ; i++) path[i] = 0;
    
    path[0] = edges[buffer_length-1].x-1;
    int prev = edges[buffer_length-1].y;
    path[1] = prev-1;
    int i = 0;
    cout << "   " << vertices[path[0]].x << " " << vertices[path[0]].y << endl;
    cout << "   " << vertices[path[1]].x << " " << vertices[path[1]].y << endl;
    while (prev != 1)
    {
        /*if(i > length-1)
         cout << "RRT: no path found " << endl;*/
        
        //int prevIndex = edges[prev-2].x;
        prev = edges[prev-2].y;
        i++;
        path[i+1] = prev-1;
        cout << "   " << vertices[path[i + 1]].x << " " << vertices[path[i + 1]].y << endl;
    }
    
    return path;
}

int RRT::isEdgeBelongs(map_point q_near, map_point q_new, int num)
{
    int Count = 5;
    int flag = 0;
    map_point v;
    v.x = q_new.x - q_near.x;
    v.y = q_new.y - q_near.y;
    double u[2];
    u[0] = v.x / sqrt(v.x*v.x + v.y*v.y);
    u[1] = v.y / sqrt(v.x*v.x + v.y*v.y);
    int delta_q2 = sqrt(v.x*v.x + v.y*v.y) / Count;
    map_point current = q_near;
    for (int i = 1; i < Count; i++)
    {
        current.x = current.x + int(delta_q2 * u[0]);
        current.y = current.y + int(delta_q2 * u[1]);
        if (static_cast<int>(map[current.y*map_W + current.x]) >= 1)
        {
            flag = 0;
            return flag;
        }
    }
    flag = 1;
    return flag;
}

double RRT::pdist2(map_point a, map_point b)
{
    return sqrt((a.x-b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

void RRT::smooth(int* path, int delta, int len)
{
    int path_smooth[100] = { 0 };
    
    int i,j,flag;
    int count = 0;
    path_smooth[0] = path[0];
    for(i = 0;i < len-2;)
    {
        count++;
        path_smooth[count] = path[i + 1];
        for (j = i + 2; j < len; j++)
        {
            flag = isEdgeBelongs(vertices[path[i]], vertices[path[j]], j-i);
            if (flag)
            {
                path_smooth[count] = path[j];
            }
            else
                break;
        }
        i = j;
    }
    i = 0;
    cout << endl << "In smooth" << endl;
    while (path_smooth[i] != 0)
    {
        cout << "   " << path_smooth[i];
        i++;
    }
}
