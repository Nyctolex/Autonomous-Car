#include "plotter.h"

    Plotter::Plotter(String *labels, int num_labels)
    {
        Labels = labels;
        NumLabels = num_labels;
    }
    void Plotter::plot(int *values)
    {
        for (int i = 0; i < NumLabels; i++)
        {
            Serial.print(Labels[i]);
            Serial.print(":");
            Serial.print(values[i]);
            Serial.print("\t");
        }
        Serial.println();
    }