#include <cstdio>

int main() {
    int age = 23;
    int height = 190;
    //int& ref; // error: declaration of reference variable 'ref' requires an initializer
    int& ref = age;
    ref = height;

    return 0;
}

