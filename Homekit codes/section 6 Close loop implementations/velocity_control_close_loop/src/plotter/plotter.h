#ifndef PLOTTER_H
#define PLOTTER_H
#include <Arduino.h>
using namespace std;
class Plotter
{
public:
    String *Labels;
    int NumLabels;
    Plotter(String *labels, int num_labels);
    void plot(int *values);
};

#endif