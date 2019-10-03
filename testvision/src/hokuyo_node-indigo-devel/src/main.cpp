#include "hokuyo_node.h"

#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
using namespace std;

int rpdiar()  //普通的函数，用来执行线程
{
    int argc;
    char **argv;
    return driver_base::main<HokuyoNode>(argc, argv, "hokuyo_node");
}

void speak()
{
    while(1)//for(int i=0;i<100;i++)
    {
    	if(dists.size()>256)cout<<dists[256]<<endl;
    }
    //cout<<dists[256]<<endl;
}


int main(int argc, char **argv)
{
	//thread th1(rpdiar);  //实例化一个线程对象th1，使用函数t1构造，然后该线程就开始执行了（t1()）
    dists.resize(512,0);
    dists[256]=256;
    thread th2(speak);
    //thread th1(rpdiar);
    
    //th1.join();
    //th2.join();
    th2.detach();//非阻塞执行，当主函数退出时候，他的线程也会退出
    //cout<<"333"<<endl;
    //th1.join();

    cout << "here is main\n\n";

    return 0;//return driver_base::main<HokuyoNode>(argc, argv, "hokuyo_node");
}