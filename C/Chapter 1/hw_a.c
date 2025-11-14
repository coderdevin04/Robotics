#include <stdio.h>

int main()
{
    int salary, rent, dearness, gross;

    printf("Enter Ramesh's basic salary ");
    scanf(" %d", &salary);

    dearness = salary * 40 / 100;
    rent = salary * 20 / 100;
    gross = salary + dearness + rent;

    printf("Ramesh's gross salary is %d", gross);

    return 0;
}