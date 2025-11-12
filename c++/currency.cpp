#include <iostream>
using namespace std;

int main()
{
    // currency converter
    int inr;
    cout << "Enter Amount to convert ₹";
    cin >> inr;
    cout << "₹" << inr << " in USD is " << inr / 85 << "\n";
    cout << "₹" << inr << " in EUR is " << inr / 96 << "\n";
    cout << "₹" << inr << " in JPY is " << inr * 1.68 << "\n";
    return 0;
}