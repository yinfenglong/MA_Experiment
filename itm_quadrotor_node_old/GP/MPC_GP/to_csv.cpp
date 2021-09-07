#include <iostream>
#include <fstream>
#include <vector>

int main(int argc, char** argv){
    std::ofstream outFile;
    outFile.open( "dataset.csv", std::ios::out );

    std::vector<int> numbers;
    for(int i=0; i<10; ++i){
        numbers.push_back(i);
    }

    /* int numbers[10] = {0,1,2,3,4,5,6,7,8,9}; */
    for(int i=0; i<10; ++i){
        outFile << numbers[i] << "," ;
    }

    outFile << std::endl;

    int iarr1[] = {1, 2, 3, 4, 5};
    int iarr2[] = {1, 3, 3, 4, 5};
    if (std::equal(std::begin(iarr1), std::end(iarr1), std::begin(iarr2))) {
        std::cout << "equal" << std::endl;
    } else {
        std::cout << "not eqaul" << std::endl;
    }

    return 0;
}
