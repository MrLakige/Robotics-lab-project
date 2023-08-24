
#ifndef __OBJECT_H__
#define __OBJECT_H__

#include <iostream>
#include <string.h>

using namespace std;

class Object{
private:
    int x, y, z;
    float probability;
    string name;

public:
    Object(string _name, float _probability, int _x, int _y);
    Object(string _name, int _x, int _y, int _z);
    ~Object();

    int getX();
    int getY();
    int getZ();
    float getProbability();
    string getName();

    friend ostream& operator<<(ostream &os, const Object &o);
    friend bool operator==(const Object &o1, const Object &o2);
    friend bool operator<=(const Object &o1, const Object &o2);
    friend bool operator<(const Object &o1, const Object &o2);
    friend bool operator>=(const Object &o1, const Object &o2);
    friend bool operator>(const Object &o1, const Object &o2);
};

ostream& operator<<(ostream &os, const Object &o);
bool operator==(const Object &o1, const Object &o2);
bool operator<=(const Object &o1, const Object &o2);
bool operator<(const Object &o1, const Object &o2);
bool operator>=(const Object &o1, const Object &o2);
bool operator>(const Object &o1, const Object &o2);

#endif
