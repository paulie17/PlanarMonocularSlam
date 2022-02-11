#include <iostream>
#include <vector>
#include <random>
#include <numeric>

#include "gnuplot-iostream.h"

using namespace std;

int main(){

    Gnuplot gp;
    random_device rd;
    mt19937 mt(rd());
    normal_distribution<double> normdist(0.,1.);

    vector<double> v0,v1;

    for(int i=0; i<1000; i++){
        v0.push_back(normdist(mt));
        v1.push_back(normdist(mt));        
    }    
    partial_sum(v0.begin(),v0.end(),v0.begin());
    partial_sum(v1.begin(),v1.end(),v1.begin());

    gp << "set title 'Graph of two random lines' \n";
    gp << "plot '-' with lines title 'v0',"
        << "'-' with lines title 'v1' \n";
    gp.send(v0);
    gp.send(v1);

    cin.get();        
}