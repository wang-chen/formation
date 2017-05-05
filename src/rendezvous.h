/*
 * rendezvous.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: jeffsan
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_


#include <iostream>
#include <string.h>
#include <iostream>
#include <math.h>
#include <map>
#include <stdio.h>
double PI = 3.141592654;
double ROBOT_ID=         1;//for different robot id
double K    =            1;//global affector
double M     =           4.0;//surround factor, should be int
#define NumberofRobots   3//number of robots
double MAX_SPEED   =     1;//max speed of robot
double LOGN2     =       0.628;//log(N,2)
double STEP       =      0.1;//Time step
double TAGET_SPEED    =  1.2;//max speed of target
double DIS_MIN      =    0.01;
using namespace std;
int ID;

class NetPack
{
public:
    double    dirNection[NumberofRobots+1][2];
    double    decision[NumberofRobots+1][2];//前进，转弯
	friend ostream & operator<< (ostream & h,NetPack & pack)
	{
			cout<<"decision"<<endl;
            for(int i=0;i<NumberofRobots+1;i++)
                 printf("%+03.3f %+03.3f %+03.3f %+03.3f\n",
                        pack.dirNection[i][0],pack.dirNection[i][1],
                         pack.decision[i][0],pack.decision[i][1]);
			return h;
	}
};

typedef NetPack netpack;
typedef NetPack Netpack;

class Rawinfo
{
public:
    double position[NumberofRobots+1][2];//0：红 1：橙 2：青 3：兰
    double direction[NumberofRobots+1][2];
	Rawinfo();
	friend ostream & operator<< (ostream & h,Rawinfo & info)
	{
		cout<<"position and orientation"<<endl;
        for(int i=0;i<NumberofRobots+1;i++)
            printf("%+03.3f %+03.3f %+03.3f %+03.3f\n",info.position[i][0],
                    info.position[i][1],info.direction[i][0],info.direction[i][1]);
		return h;
	}
};
Rawinfo::Rawinfo()
{
	memset(position, 0, sizeof(position));
	memset(direction, 0, sizeof(direction));
}
typedef Rawinfo rawInfo;
typedef Rawinfo RawInfo;
typedef Rawinfo rawinfo;

Netpack pack_test,pack_stop;
rawinfo info_test;
void init_testpack()
{
    pack_test.decision[1][0]=1;
    pack_test.decision[1][1]=300;
    pack_test.decision[2][0]=1;
    pack_test.decision[2][1]=300;
    pack_test.decision[3][0]=-1;
    pack_test.decision[3][1]=300;

    memset(&pack_stop,0,sizeof(pack_stop));

    info_test.position[0][0]=0;
    info_test.position[0][1]=0;
    info_test.position[1][0]=-1;
    info_test.position[1][1]=0;
    info_test.position[2][0]=-1;
    info_test.position[2][1]=-1;
    info_test.position[3][0]=-1;
    info_test.position[3][1]=1;

    info_test.direction[0][0]=1;
    info_test.direction[0][1]=0;
    info_test.direction[1][0]=1;
    info_test.direction[1][1]=0;
    info_test.direction[2][0]=1;
    info_test.direction[2][1]=0;
    info_test.direction[3][0]=1;
    info_test.direction[3][1]=0;
}

struct ss_Pack
{
    double dis[NumberofRobots+1];//ID:self
    double friend_angle[NumberofRobots+1][2];//0:left(clockwise),1:right(unclockwise)
    double hunting_angle[NumberofRobots+1][2];//0left,1:right
    double target_angle[NumberofRobots+1];//for zigbee: return posistion for right target return negtive for left target
};

ss_Pack a[3];
class Hunting
{
public:
    Hunting();
    double Getangle(double ,double ,double ,double ,double );
private:
    double lamda;//angle between two max direction
    double deta;//surrounding factor
    double gamma;//capture factor
    double k;//global factor
    double m;//surrounding factor ,int
    double decision;//decision angle, according to target, not the final return value.
};

Hunting::Hunting()
{
    lamda = 0;
    deta  = 0;
    gamma = 0;
    k     = K;
    m     = M;
    decision = 0;
}

double Hunting::Getangle(double clock,double unclock,double dis,double dis_sum,double maxspeed)
{
    //C++can not compute negtive expential, here also compute turning angle(left right)
    int flag=0;
    if(clock-unclock<0)//clockwise
    {
        deta  = pow(-(clock-unclock)/(2*PI),1.0/m);
        flag=1;
    }
    else //unclockwise
    {
        deta  = pow((clock-unclock)/(2*PI),1.0/m);
        flag=-1;
    }

    gamma    = sin(PI*pow(1.0*dis/dis_sum,LOGN2));
    lamda    = 4.0/9*PI;//acos(maxspeed*STEP/dis);

    decision = lamda*(1-pow(2.71828,-1.0*k*deta*gamma));
    return flag*decision;
}


class Robot:Hunting
{
public:
    NetPack  MakeDeci(Rawinfo);//only interface, return turning angel,left is negtive right is position
    Robot();
    bool captured;
protected:
    NetPack pack;
    ss_Pack ss_pack;//123robots
    Hunting A,B,C;
    ss_Pack Calculate(double pos[][2],double dir[][2]);//internal interface
    double  maxspeed;
    double AbsoluteAngle(int x,int y);
    double VecAngle(double a[2],double b[2]);
    void PointMinus(double dest[2],double a[2],double b[2]);
    double EuclidNorm(double a[2],double b[2]);
    char* flag_capture;
    //CvPoint position_capture;
    //CvFont font_capture;
};

Robot::Robot()
{
    maxspeed      = MAX_SPEED;
    captured=false;
    flag_capture="Captured";
    //cvInitFont(&font_capture,CV_FONT_HERSHEY_DUPLEX,1.0,1.0,0.0,2,8);
}

class myFilter{
public:
    myFilter():state(false){}
    bool operator() (int in,int dis_low,int dis_high){
        if (state && in>dis_high) state=false;
        if (!state && in<dis_low) state=true;
        return state;
    }
    bool last_one() {return state;}
    void set_false() {state=false;}
private:
    bool state;
}DisMinFilter[4];

NetPack Robot::MakeDeci(Rawinfo rawinfo)
{
    int simple=0;
    ss_pack=Calculate(rawinfo.position, rawinfo.direction);//get global information

    double dis_sum=ss_pack.dis[1]+ss_pack.dis[2]+ss_pack.dis[3];
    double angle,deci;
    for(int i=1;i<=NumberofRobots;i++)
    {
        deci=Getangle(
        ss_pack.friend_angle[i][0],ss_pack.friend_angle[i][1],
        ss_pack.dis[i],dis_sum,maxspeed);

        angle=ss_pack.target_angle[i]+deci;

        pack.decision[i][1] = -angle/PI*180;
        pack.decision[i][0] = 1;

        DisMinFilter[i](ss_pack.dis[i],DIS_MIN,DIS_MIN+2);

        if(rawinfo.position[0][0]==0||DisMinFilter[i].last_one()
         ||rawinfo.position[i][0]==0||rawinfo.position[i][1]==0)
         //||rawinfo.direction[i][0]==0||rawinfo.direction[i][1]==0){
        {
            pack.decision[i][0]=0;
            if(!(rawinfo.position[0][0]!=0 && rawinfo.position[0][1]!=0
               && rawinfo.position[i][0]!=0))
               DisMinFilter[i].set_false();
        }
    }

    captured=DisMinFilter[1].last_one()||DisMinFilter[2].last_one()||DisMinFilter[3].last_one(); //if any three is captured.
    if (captured){
        if(rawinfo.position[0][0]<=320 && rawinfo.position[0][1]<=240)
        {
            //position_capture.x=400;position_capture.y=70;
            //cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
        }
        else if(rawinfo.position[0][0]<=320 && rawinfo.position[0][1]>=240)
        {
            //position_capture.x=400;position_capture.y=410;
            //cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
        }
        else if(rawinfo.position[0][0]>=320 && rawinfo.position[0][1]<=240)
        {
            //position_capture.x=70;position_capture.y=70;
            //cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
        }
        else if(rawinfo.position[0][0]>=320 && rawinfo.position[0][1]>=240)
        {
            //position_capture.x=70;position_capture.y=410;
            //cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
        }
        //cvShowImage("Global",contours);
    }
    return pack;

}

ss_Pack Robot::Calculate(double pos[][2],double dir[][2])
{
    int i,j;
    ss_Pack res;
    res.dis[0]=0;	//useless...
    for (i=1;i<=NumberofRobots;i++)
        res.dis[i]=EuclidNorm(pos[i],pos[0]);
    res.target_angle[0]=0;
    res.friend_angle[0][0]=0;
    res.friend_angle[0][1]=0;
    //Haunting Angle
    for (i=0;i<=NumberofRobots;i++)
    {
        res.hunting_angle[i][0]=2*PI/3;
        res.hunting_angle[i][1]=2*PI/3;
    }
    //Target Angle
    for (i=1;i<=NumberofRobots;i++){
        double tpoint[2];double tmp;
        PointMinus(tpoint,pos[0],pos[i]);	//tpoint=pos.tar-pos.me
        tmp=-VecAngle(tpoint,dir[i]);
        if (i==2&&0){
            cout<<"i="<<i<<"dir="<<dir[i][0]<<","<<dir[i][1];
            cout<<"\tTar="<<pos[0][0]<<","<<pos[0][1]<<endl;
            cout<<"\tTpoint="<<tpoint[0]<<","<<tpoint[1]<<"\tang="<<tmp<<endl;
        }
        res.target_angle[i]=tmp;
    }
    //Friend Angle, based on target
    double tbase[2];
    multimap<double,int> FriendMap;
    PointMinus(tbase,pos[1],pos[0]);	//base vec:me0->tar
    FriendMap.insert(std::pair<double,int>(0,1));
    for (i=2;i<=NumberofRobots;i++){
        double tpoint[2];double ang;
        PointMinus(tpoint, pos[i], pos[0]);
        ang=VecAngle(tpoint,tbase);		//tpoint.Angle-tbase.Angle,anticounter angle from tbase to tpoint
        FriendMap.insert(std::pair<double,int>(ang,i));
    }
    multimap<double,int>::iterator iter,iter2;
    //show FriendMap
    /*for (iter=FriendMap.begin();iter!=FriendMap.end();iter++)
        cout<<"id="<<iter->second<<",ang="<<iter->first<<endl;*/
    iter=FriendMap.begin();iter2=iter;iter2++;
    double val;
    for (;iter2!=FriendMap.end();){
        val=iter2->first-iter->first;
        if (val<0) throw;
        res.friend_angle[iter->second][1]=val;
        res.friend_angle[iter2->second][0]=val;
        iter++;iter2++;
    }
    iter2=FriendMap.begin();
    val=iter2->first-iter->first;
    if (val<0) val+=2*PI;
    res.friend_angle[iter->second][1]=val;	//last one's right
    res.friend_angle[iter2->second][0]=val;	//first one's left
    return res;
}

double Robot::AbsoluteAngle(int x,int y){
    static double ang;
    if (y==0) return x>0?0:PI;
    if (x==0) return y>0?(PI/2):(-PI/2);
    ang=atan(y/(float)x);
    if (ang<0) ang+=2*PI;
    ang=x>0?ang:(ang+PI);
    if (ang>PI) return ang-2*PI;
    if (ang<-PI) return ang+2*PI;
    return ang;
}

double Robot::VecAngle(double a[2],double b[2]){
    double angA=AbsoluteAngle(a[0],a[1]);
    double angB=AbsoluteAngle(b[0],b[1]);
    angB=angA-angB;
    if (angB>PI) return angB-2*PI;
    if (angB<-PI) return angB+2*PI;
    return angB;
}

void Robot::PointMinus(double dest[2],double a[2],double b[2]){
    dest[0]=a[0]-b[0];
    dest[1]=a[1]-b[1];
}

double Robot::EuclidNorm(double a[2],double b[2]){
    double dx=a[0]-b[0];
    double dy=a[1]-b[1];
    dx*=dx;dy*=dy;
    return sqrt(dx+dy);
}
//namespace rendezvous
#endif /* GLOBAL_H_ */
