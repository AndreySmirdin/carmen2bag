#include <iostream>
#include "converter.h"

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cout << "Usage: carmen2bag <input filename> <output filename> <parameters filename>" << "\n";
        return 0;
    }
    ros::init(argc, argv, "carmen2bag");
    ros::NodeHandle nh;


    try {
        Converter converter(argv[1], argv[2], argv[3]);
        converter.convert();
    }
    catch(std::logic_error e) {
        std::cout << e.what();
    }
    return 0;
}

