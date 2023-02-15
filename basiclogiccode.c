#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.14159265

int main()
{
    int posG[] = {0, 0, 1000};
    int posR[] = {(- 90 + rand() % 180), (-180 rand() % 360), (3000)};
    int angularpos[] = {(-360 + rand() % 720), (-360 + rand() % 720), (-360 + rand() % 720)};
    
    int resultant[] = {posR[0] - posG[0], posR[1] - posG[1], posR[2] - posG[2]};
    
    float resultant_mag = sqrt(pow(resultant[0],2) + pow(resultant[1],2) + pow(resultant[2],2));
    float angle_needed[] = {acos(resultant[0]/resultant_mag) * 180.0 / PI, acos(resultant[1]/resultant_mag) * 180.0 / PI, acos(resultant[2]/resultant_mag) * 180.0 / PI};
    
    if (angle_needed[0] > angularpos[0]){
        printf("Turn positive in \n");
    } else {
        printf("Turn negative in x \n");
    }

    if (angle_needed[1] > angularpos[1]){
        printf("Turn positive in y \n");
    } else {
        printf("Turn negative in y \n");
    }
    
      if (angle_needed[2] > angularpos[2]){
        printf("Turn positive in z\n");
    } else {
        printf("Turn negative in z\n");
    }
    
    return 0;
}

