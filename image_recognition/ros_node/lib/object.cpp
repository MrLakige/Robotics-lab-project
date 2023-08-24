#include "object.h"
#include <iostream>
#include <string.h>

using namespace std;


Object::Object(string _name, float _probability, int _x, int _y ){
    x=_x;
    y=_y;
    z=0;
    probability=_probability;
    name=_name;
}

Object::Object(string _name, int _x, int _y, int _z ){
    x=_x;
    y=_y;
    z=_z;
    name=_name;
    probability=-1;
}

Object::~Object(){
}

int Object::getX(){
    return x;
}

int Object::getY(){
    return y;
}

int Object::getZ(){
    return z;
}

float Object::getProbability(){
    return probability;
}

string Object::getName(){
    return name;
}

ostream& operator<<(ostream &os, const Object &o){
    return os<<"["<<o.name<<","<<o.x<<","<<o.y<<","<<o.z<<"]";
}

bool operator==(const Object &o1, const Object &o2){
    if(!strcmp(o1.name.c_str(), o2.name.c_str())) return false;
    if(o1.x!= o2.x) return false;
    if(o1.y!= o2.y) return false;
    if(o1.y!= o2.y) return false;
    if(o1.probability!= o2.probability) return false;
    return true;
}

bool operator<=(const Object &o1, const Object &o2){
    if(!strcmp(o1.name.c_str(), o2.name.c_str())>0) return false;
    if(o1.x > o2.x) return false;
    if(o1.y > o2.y) return false;
    if(o1.y > o2.y) return false;
    if(o1.probability > o2.probability) return false;
    return true;
}
bool operator<(const Object &o1, const Object &o2){
     if(!strcmp(o1.name.c_str(), o2.name.c_str())>=0) return false;
    if(o1.x >= o2.x) return false;
    if(o1.y >= o2.y) return false;
    if(o1.y >= o2.y) return false;
    if(o1.probability >= o2.probability) return false;
    return true;
}
bool operator>=(const Object &o1, const Object &o2){
    return !(o1 < o2);
}
bool operator>(const Object &o1, const Object &o2){
    return !(o1 <= o2);
}