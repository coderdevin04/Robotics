#include <stdio.h>

int main() {
    int Num;
    printf("Enter your desired 5-digit number & see the magic happens \n");
    scanf("%d",&Num);

    Num = Num +1+10+100+1000+10000;

    printf("The Magic %d", Num);

    return 0;
}