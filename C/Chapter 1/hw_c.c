#include <stdio.h>

int main()
{
    int a, b, c, d, e ,sb;
    printf("Enter test marks of Subject A ");
    scanf("%d", &a);

    printf("Enter test marks of Subject B ");
    scanf("%d", &b);

    printf("Enter test marks of Subject C ");
    scanf("%d", &c);

    printf("Enter test marks of Subject D ");
    scanf("%d", &d);

    printf("Enter test marks of Subject E ");
    scanf("%d", &e);

    sb = a + b + c + d + e;

    printf("Your Aggregate Marks are %d out of 500\n", sb);
    printf("Your Percentage Marks are %f out of 100.00", sb * 100.00 / 500.00);

    return 0;
}