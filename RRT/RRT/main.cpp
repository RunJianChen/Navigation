//
//  main.cpp
//  RRT
//
//  Created by Chen~_~RJ on 2018/11/29.
//  Copyright © 2018 Chen~_~RJ. All rights reserved.
//

#include <iostream>
#include "RRT(origin).hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

//删除字符串中空格，制表符tab等无效字符
std::string Trim(std::string& str)
{
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0,str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

int main(int argc, const char * argv[])
{
    std::ifstream fin("/Users/chenrj/Desktop/机器人学/轮式/大作业/map.csv"); //打开文件流操作
    std::string line;
    int height=240,width=300,count_lin=0;
    unsigned char *map_data = new unsigned char[height*width];
    
    while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
    {
        int count_col = 0;
        std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        std::vector<std::string> fields; //声明一个字符串向量
        std::string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
            int temp;
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields
            temp = std::stoi(field);
            map_data[count_lin*width+count_col] = static_cast<unsigned char> (temp);
            count_col++;
        }
        count_lin++;
    }
    
    int start_x = 0 , start_y = 0, end_x = 0 , end_y = 0;
    
    RRT* new_RRT = new RRT(height,width,map_data,114,200,130,130);
    
    new_RRT->Path_Generator();
    
    return 0;
}
