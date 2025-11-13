#include <stdio.h>

int main() {
    int dist;
    printf("Enter the distance(in km) between cities ");
    scanf("%d",&dist);

    //dist= dist*1000;

    printf("Distance in meters %d\n",dist*1000);
    printf("Distance in feet %f\n",dist*3280.84);
    printf("Distance in inches %f\n",dist*39370.1);
    printf("Distance in centimeters %d\n",dist*100000);
    

    return 0;
}