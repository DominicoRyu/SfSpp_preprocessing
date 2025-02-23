#ifndef DATA_PATH_H
#define DATA_PATH_H

// Pot 선택: 하나만 정의되어야 함
#define POT_A
//#define POT_B
//#define POT_C
//#define POT_D
//#define POT_E
//#define POT_F
//#define POT_G
//#define POT_H
//#define POT_I
//#define POT_J

#include <string>

//############################################ Pot ID (MeshProcessing & EdgelineExtraction) ############################################//
#ifdef POT_A
    static const std::string potID = "A";
#elif defined(POT_B)
    static const std::string potID = "B";
#elif defined(POT_C)
    static const std::string potID = "C";
#elif defined(POT_D)
    static const std::string potID = "D";
#elif defined(POT_E)
    static const std::string potID = "E";
#elif defined(POT_F)
    static const std::string potID = "F";
#elif defined(POT_G)
    static const std::string potID = "G";
#elif defined(POT_H)
    static const std::string potID = "H";
#elif defined(POT_I)
    static const std::string potID = "I";
#elif defined(POT_J)
    static const std::string potID = "J";
#else
    #error "No POT defined! Please define one POT (e.g., POT_A)."
#endif

inline std::string getMeshDatasetPath(const std::string& potID) {
    return "../Dataset/Mesh/Pot_" + potID + "/";
}

inline std::string getPointDatasetPath(const std::string& potID) {
    return "../Dataset/Point/Pot_" + potID + "/";
}

inline std::string getBreaklineDatasetPath(const std::string& potID) {
    return "../Dataset/Breaklines/Pot_" + potID + "/";
}

inline std::string getSurfaceDatasetPath(const std::string& potID) {
    return "../Dataset/Surfaces/Pot_" + potID + "/";
}

inline std::string tempPath() {
    return "../Temp/";
}

inline std::string tempDataPath(const std::string& potID) {
    return "../Temp/Data/Pot_" + potID + "/";
}

inline std::string tempIntermediatePath(const std::string& potID) {
    return "../Temp/Temp/Pot_" + potID + "/";
}

#endif // DATA_PATH_H
