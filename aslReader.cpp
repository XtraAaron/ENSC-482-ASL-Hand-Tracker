#include <winsock2.h> // Winsock header
#include <ws2tcpip.h> // Extended Winsock stuff, needed for some helper functions
#include <cstdio> // Using printf not cout for this one
#include <vector> // Vector cuz i dont like arrays
#include <string> // String
#include <iostream>

enum wristState { // Enum for the wrist state, makes it ez to track
    BASE, // Base state
    ROLL, // Roll state
    ROLL_AND_PITCH, // RP state
    YAW, // Yaw state
    REVERSE_ROLL, // Number state
    UNKNOWN // We dont know what state its in rn, if its this dont do anything
};

enum class CurlState {
    FULL_CURL,
    FULL_EXTEND,
    UNKNOWN };
// Future improvment for sure, is that ideally we would have a function perform classification
// If we used this, we could easily switch to switch instead of many ifs


#pragma comment(lib, "Ws2_32.lib") // Autolink Winsock if using MSVC
// Not needed if runing g++ tho

// Note, the .bat file only works with g++, need smth different if not using g++

#define TRACKED_BONE 13 // Goes from 0 to 14

/*
Bone	Index
Palm	0
Thumb1	1
Thumb2	2
Index1	3
Index2	4
Middle1	6
Middle2	7
Ring1	9
Ring2	10
Pinky1	12
Pinky2	13
Pinky3	14
*/

#define UDP_PORT 5053 // Output port
#define NUM_BONES 15 // Number of bones
#define NUM_FLOAT 45 // Would perefer double byt python packs in floats

// All bounds hand tuned
CurlState classifyIndex1(const float& I1_rotation) {
    if (I1_rotation >= -0.500 && I1_rotation <= -0.100) return CurlState::FULL_EXTEND; // Full extend state
    else if (I1_rotation >= -1.610 && I1_rotation <= 0.525) return CurlState::FULL_CURL;
    else return CurlState::UNKNOWN; // Unknown state
}

CurlState classifyIndex2(const float& I2_rotation) {
    if (I2_rotation >= -1.600 && I2_rotation <= -0.750) return CurlState::FULL_CURL; // Full curl state
    else if (I2_rotation >= -0.725 && I2_rotation <= 0.000) return CurlState::FULL_EXTEND; // Full curl state
    else return CurlState::UNKNOWN;
}

CurlState classifyIndex3(const float& I3_rotation) {
    if (I3_rotation >= -1.600 && I3_rotation <= -0.700) return CurlState::FULL_CURL; // Full curl state
    else if (I3_rotation >= -0.675 && I3_rotation <= 0.000) return CurlState::FULL_EXTEND; // Full curl state
    else return CurlState::UNKNOWN;
}

CurlState classifyMiddle1(const float& M1_rotation) {
    if (M1_rotation >= -0.525 && M1_rotation <= -0.070) return CurlState::FULL_EXTEND; 
    else if (M1_rotation >= -1.610 && M1_rotation <= -0.450) return CurlState::FULL_CURL; 
    else return CurlState::UNKNOWN;
}

CurlState classifyMiddle2(const float& M2_rotation) {
    if (M2_rotation >= -0.570 && M2_rotation <= 0.000) return CurlState::FULL_EXTEND; 
    else if (M2_rotation >= -1.610 && M2_rotation <= -0.575) return CurlState::FULL_CURL; 
    else return CurlState::UNKNOWN;
}

CurlState classifyRing1(const float& R1_rotation) {
    if (R1_rotation >= -0.475 && R1_rotation <= -0.075) return CurlState::FULL_EXTEND; // Not measured by hand, if needed change this
    else if (R1_rotation >= -1.610 && R1_rotation <= -0.525) return CurlState::FULL_CURL; // Not measured by hand, if needed change this
    else return CurlState::UNKNOWN;
}
CurlState classifyRing2(const float& R2_rotation) {
    if (R2_rotation >= -0.475 && R2_rotation <= 0.000) return CurlState::FULL_EXTEND; // Not measured by hand, if needed change this
    else if (R2_rotation >= -1.610 && R2_rotation <= -0.525) return CurlState::FULL_CURL; // Not measured by hand, if needed change this
    else return CurlState::UNKNOWN;
}

CurlState classifyPinky1(const float& P1_rotation) {
    if (P1_rotation >= -0.700 && P1_rotation <= -0.150) return CurlState::FULL_EXTEND; 
    else if (P1_rotation >= -1.610 && P1_rotation <= -0.710) return CurlState::FULL_CURL; // Not measured by hand, if needed change this
    else return CurlState::UNKNOWN;
}

CurlState classifyPinky3(const float& P3_rotation) {
    if (P3_rotation >= -0.425 && P3_rotation <= -0.000) return CurlState::FULL_EXTEND; 
    else if (P3_rotation >= -1.610 && P3_rotation <= -0.650) return CurlState::FULL_CURL; // Not measured by hand, if needed change this
    else return CurlState::UNKNOWN;
}
// This stuff partially used, check notes on why. Much easier on full curl and extend, due to how often they used vs semi
// Also why no yaw function.
// Wrist is specific, so it just gets mashed into its own function anyways

// May change, using it a lot in base change comment if needed


int frameCounter;


const std::vector<std::string> BONE_NAMES = {
    "Palm", "Thumb1", "Thumb2",
    "Index1", "Index2", "Index3",
    "Middle1", "Middle2", "Middle3",
    "Ring1", "Ring2", "Ring3",
    "Pinky1", "Pinky2", "Pinky3"
}; // Use this too to track


void printBoneRotation(const std::vector<std::vector<float>>& rotations, int boneIndex) {
    if (boneIndex < 0 || boneIndex >= (int)rotations.size()) {
        printf("Invalid bone index: %d\n", boneIndex);
        return;
    }

    const auto& rotation = rotations[boneIndex];
    printf("%s rot: %.3f %.3f %.3f\n", BONE_NAMES[boneIndex].c_str(), rotation[0], rotation[1], rotation[2]);
} // Debugging function, used to get the information for finding rotational data
// Not at all important, just prints XYZ and respective tracked bone


wristState returnWristState(const std::vector<std::vector<float>>& returnWristState_RotationMatrix) {
    // Can't use switch due to range
    // First comparison is for min coords, second max coords
    if (returnWristState_RotationMatrix[0][1] >= 0.700 && returnWristState_RotationMatrix[0][1] <= 2.200) { // Roll state, track Y
        //std::cout << "This is in roll state hot dog flavoured water\n\n";
        return ROLL;
    } // Roll placed at top, as it helps deal with base overlapping with its boundry sometime
    else if (returnWristState_RotationMatrix[0][1] >= 2.200 && returnWristState_RotationMatrix[0][1] <= 3.100) { // Reverse state, track Y
        //std::cout << "This is in Reverse State state\n\n";
        return REVERSE_ROLL;
    } // Basically divide roll into 2 sections, reversed wrist and sideways
    else if (returnWristState_RotationMatrix[0][0] >= -0.450 && returnWristState_RotationMatrix[0][0] <= 0.450 && // X
             returnWristState_RotationMatrix[0][1] >= -0.575 && returnWristState_RotationMatrix[0][1] <= 0.575 && // Y
             returnWristState_RotationMatrix[0][2] >= -0.575 && returnWristState_RotationMatrix[0][2] <= 0.575) { // Z
        //std::cout << "This is in base state\n\n";
        return BASE; // Base state
    }
    else if (returnWristState_RotationMatrix[0][0] >= 0.600 && returnWristState_RotationMatrix[0][0] <= 1.00 && // Track everything individually to remove overlap
             returnWristState_RotationMatrix[0][1] >= -2.000 && returnWristState_RotationMatrix[0][1] <= -1.000 &&
             returnWristState_RotationMatrix[0][2] >= -0.500 && returnWristState_RotationMatrix[0][2] <= 0.100) { // Roll and pitch state
        //std::cout << "This is in roll and pitch state\n\n";
        return ROLL_AND_PITCH;
    } // Put roll and pitch b4 yaw since yaw is more general, filter out the more specilized state first
    else if (returnWristState_RotationMatrix[0][0] >= 0.000 && returnWristState_RotationMatrix[0][0] <= 1.250) { // Yaw state, track X
        //std::cout << "This is in yaw state\n\n";
        return YAW;
    }
    else {
        //std::cout << "Current state is unknown\n\n";
        return UNKNOWN;
    }     
} // This function is the first stage of the tree, determine what kinda of movment is being undertaken
// All bounds were found manually, more tuning could be done to get better results. For now should be good enough


char rollTree(const std::vector<std::vector<float>>& rollTree_rotationMatrix){ // ω2 
    // Star with I1
                // std::cout
                // << " I1: " << rollTree_rotationMatrix[3][2]
                // << " M1: " << rollTree_rotationMatrix[6][2]
                // //<< " I2: " << rollTree_rotationMatrix[4][2] 
                // << "\n";      // Debug stuff
    if (rollTree_rotationMatrix[3][2] >= -1.000 && rollTree_rotationMatrix[3][2] <= -0.700) { // I1 semi-curl;
        return 'o'; 
    } // Detect O     

    else if (rollTree_rotationMatrix[3][2] >=  -0.250 && rollTree_rotationMatrix[3][2] <= -0.050 && // I1 semi-extend
            rollTree_rotationMatrix[6][2] >= -0.180 && rollTree_rotationMatrix[6][2] <= -0.000) { // M1 semi-extend
        return 'c';
    } // Detects C   

    if (classifyIndex1(rollTree_rotationMatrix[3][2]) == CurlState::FULL_EXTEND) { // I1 FE
        if (rollTree_rotationMatrix[6][2] >= -0.950 && rollTree_rotationMatrix[6][2] <= -0.300) { // M1 semi-curl 
            return 'd';
        } // Detects D 

        else if (classifyMiddle1(rollTree_rotationMatrix[6][2]) == CurlState::FULL_CURL && // M1 FC
                classifyIndex2(rollTree_rotationMatrix[4][2]) == CurlState::FULL_CURL) { // I2 FC
            return 'x';
        } // Detects X

        else {      
            return '+';
        } // Else not a letter we are dealing with
    }  
    else {
        return '+'; // Let "+" be the nothing detected value
    } // None of the above. No letter produced
    return '-'; // Let '-' be the error value if something breaks completly
}
// Needs M1, I1, I2
// C and O are the only "partial" curls used in the tree
// Covers the Roll tree (see image)


char yawTree(const std::vector<std::vector<float>>& yawTree_rotationMatrix) {
    // Check M1 first (first branch)

    if (classifyMiddle1(yawTree_rotationMatrix[6][2]) == CurlState::FULL_EXTEND) { // M1 FE (H, P)
        if (classifyMiddle2(yawTree_rotationMatrix[7][2]) == CurlState::FULL_EXTEND && // M2 FE
                yawTree_rotationMatrix[6][0] >= -0.325 && yawTree_rotationMatrix[6][0] <= 0.450) { // M1 Yaw
            return 'h';
        } // H

        else if (classifyIndex1(yawTree_rotationMatrix[3][2]) == CurlState::FULL_EXTEND) { // I1 FE 
            return 'p'; 
        } // P

        else {
            return '+';
        } // Not H or P
        
    }

    else if (classifyMiddle1(yawTree_rotationMatrix[6][2]) == CurlState::FULL_CURL) { // M1 FC (G, J 10)
        if (classifyPinky1(yawTree_rotationMatrix[12][2]) == CurlState::FULL_EXTEND) { // Checks P1 (Not using P2, not needed)
            return 'j';
        } // J
        
        else if (classifyIndex1(yawTree_rotationMatrix[3][2]) == CurlState::FULL_EXTEND) { // I1 FE
            return 'g';
        } // G

        else if (yawTree_rotationMatrix[1][0] >= -0.825 && yawTree_rotationMatrix[1][0] <= -0.425 ) { // Thump Pos X
            return '0'; // Technically should be 10, but yea its becoming represented by a 0
        } // 10
            
        else {
            return '-';
        } // Unknown letter
    }

    return '|';
} // G H J P 10
// Uses I1 M1 M2 P1 P2
// Check J last, cuz it only checks pinky
// Add J to use M1 too (FC)


char reverseTree(const std::vector<std::vector<float>>& reverseTree_rotationMatrix) { //
    // std::cout
    // << " P1: " << reverseTree_rotationMatrix[12][2]
    // << " M1: " << reverseTree_rotationMatrix[6][2]
    // << " I1: " << reverseTree_rotationMatrix[3][2]
    // << "\n"; // Debug stuff  

    if (reverseTree_rotationMatrix[1][0] >= -0.825 && reverseTree_rotationMatrix[1][0] <= -0.050 ) { // T1 Out (3, 5)
        // T1 stuff maybe possible to make its own function? I dont want to rn tho when its uncertain (esp with like a, s and t)
        if (reverseTree_rotationMatrix[12][2] >= -1.100 && reverseTree_rotationMatrix[12][2] <= -0.550 && // P1 FE
            reverseTree_rotationMatrix[6][2] >= -0.850 && reverseTree_rotationMatrix[6][2] <= -0.125 && // M1 FE
            reverseTree_rotationMatrix[3][2] >= -0.725 && reverseTree_rotationMatrix[3][2] <= -0.200) { // I1 FE
            return '3';
        } // 3

        else if (reverseTree_rotationMatrix[12][2] >= -0.600 && reverseTree_rotationMatrix[12][2] <= -0.400 && // P1 FE
                reverseTree_rotationMatrix[6][2] >= -0.850 && reverseTree_rotationMatrix[6][2] <= 0.000 && // M1 FE
                reverseTree_rotationMatrix[3][2] >= -0.725 && reverseTree_rotationMatrix[3][2] <= -0.175) { // I1 FE
        
            return '5'; // Probably should have a check here tbh, but if it detects a 5 good enough for me
            // Def future work material
        } // 5

        else {
            return '-';
        } // Unknown
    }

    else { // T1 in (1, 2, 4)
        if (reverseTree_rotationMatrix[6][2] >= -0.475 && reverseTree_rotationMatrix[6][2] <= 0.250) { // M1 FE
            if (reverseTree_rotationMatrix[12][2] >= -0.500 && reverseTree_rotationMatrix[12][2] <= -0.425 // P1 FE
            ) { // P1 FE
                return '4';
            } // 4

            else if (reverseTree_rotationMatrix[3][2] >= -0.375 && reverseTree_rotationMatrix[3][2] <= -0.075) { // I1 FE
                return '2';
            }   
        } // 2, 4

        else if (reverseTree_rotationMatrix[3][2] >= -0.350 && reverseTree_rotationMatrix[3][2] <= -0.125) { // I1
            return '1';
        } // 1

        else {
            return '+';
        }
    } // Since thumb is hidden, dont want to add a constraint, just assume a diff number

    return '|'  ;
} // 



// Depth of 5 total
char baseTreeD5(const std::vector<std::vector<float>>& baseTreeD5_rotationMatrix, int combinedResult) {
    //std::cout
    //<< " T1 X: " << baseTreeD5_rotationMatrix[1][0]
    //<< " T1 Z: " << baseTreeD5_rotationMatrix[1][2]
    //<< "\n"; // Debug stuff     
    switch (combinedResult) {
        case 1: 
        // B 7 8
        // I1 FE, R1 FE, P1 FE, I2 FE
        // Individual, B is last case, 7 checks R2 curl, 8 checks M2 curl, B checks both straight
        //std::cout << "Case 1\n";
            if (classifyRing2(baseTreeD5_rotationMatrix[10][2]) == CurlState::FULL_CURL) { // Curl R2
                return '7';
            }

            else if (classifyMiddle2(baseTreeD5_rotationMatrix[7][2]) == CurlState::FULL_CURL) { // Curl M2
                return '8';
            }

            else if (classifyRing2(baseTreeD5_rotationMatrix[10][2]) == CurlState::FULL_EXTEND && // R2 Extended
                     classifyMiddle2(baseTreeD5_rotationMatrix[7][2]) == CurlState::FULL_EXTEND) { // M2 extended
                return 'b';
            }

            else { // Unknown
                return '%';
            }

            break;
        case 2: 
        // E F 
        
        // I1 FE, R1 FE, P1 FE, I2 FC
        // Use P3
        //std::cout << "Case 2\n";
            if (classifyPinky3(baseTreeD5_rotationMatrix[14][2]) == CurlState::FULL_CURL) { // P2 FC
                return 'e';
            }

            else if (classifyPinky3(baseTreeD5_rotationMatrix[14][2]) == CurlState::FULL_EXTEND) { // P2 FE
                return 'f';
            }

            else { // Unknown
                return '%';
            }   

            break;
        case 3: 
        // K V
        // Use thumb pos
        //std::cout << "Case 3\n";
            if (baseTreeD5_rotationMatrix[1][0] >= 0.200 && baseTreeD5_rotationMatrix[1][0] <= 0.350) { // Need fresh one
                return 'k'; // Add p1 curl to avoid 7 overlap, when testing make sure to test 7
            }

            else if (baseTreeD5_rotationMatrix[1][0] >= 0.360 && baseTreeD5_rotationMatrix[1][0] <= 0.750) { // Reuse A S
                return 'v';
            }

            else { // Unknown
                return '%';
            }   
                       
            break;
        default:
            return '%';
            break;

    }
}

char baseTreeD4(const std::vector<std::vector<float>>& baseTreeD4_rotationMatrix, int combinedResult) {
    // std::cout
    // << " T1 X: " << baseTreeD4_rotationMatrix[1][0]
    // << " T1 Z: " << baseTreeD4_rotationMatrix[1][2]
    // << "\n"; // Debug stuff     
    //std::cout << "I1 " << baseTreeD4_rotationMatrix[3][0] << '\n';
    switch (combinedResult){
        case 1: // I1 FE, R1 FE, P1 FE
        // B E F 7 8
        //std::cout << "Case 1\n";
        //std::cout << "I2 rot: " << baseTreeD4_rotationMatrix[4][2] << '\n';
            if (classifyIndex2(baseTreeD4_rotationMatrix[4][2]) == CurlState::FULL_EXTEND) { // I2 FE
                //std::cout << "I1 FE, R1 FE, P1 FE, B 7 8\n";
                return baseTreeD5(baseTreeD4_rotationMatrix, 1);
            } // B 7 8

            else if (classifyIndex2(baseTreeD4_rotationMatrix[4][2]) == CurlState::FULL_CURL) { // I2 FC
                //std::cout << "I1 FE, R1 FE, P1 FE, E F\n";
                return baseTreeD5(baseTreeD4_rotationMatrix, 2);
            } // E F

            else { // Unknown
                return '#';
            }

            break;

        case 2: // I1 FE, R1 FE, P1 FC
        // R W
        //std::cout << "Case 2\n";        
            if (classifyRing2(baseTreeD4_rotationMatrix[10][2]) == CurlState::FULL_EXTEND) { // R2 FE
                //std::cout << "I1 FE, R1 FE, P1 FC, W\n";
                return 'w';
            } // W or 6

            // else if (classifyRing2(baseTreeD4_rotationMatrix[10][2]) == CurlState::FULL_CURL) { // R2 FC
            //     std::cout << "I1 FE, R1 FE, P1 FC, R\n";
            //     return '@';
            // } // R

            // R overlapped with case 4 so moving it there

            else { // Unknown
                return '#';
            }

            break;

        case 3: // I1 FE, F1 FC, M1 FC
        // L Z
        // Reuse x curl for z
        //std::cout << "Case 3\n";  
        //std::cout << "I3 rot: " << baseTreeD4_rotationMatrix[5][2] << '\n';
            if (classifyIndex3(baseTreeD4_rotationMatrix[5][2]) == CurlState::FULL_CURL) { // I3 FC
                //std::cout << "I1 FE, F1 FC, M1 FC, Z\n";
                return 'z';
            } // Z

            else if (classifyIndex3(baseTreeD4_rotationMatrix[5][2]) == CurlState::FULL_EXTEND) { //I2 FE
                //std::cout << "I1 FE, F1 FC, M1 FC, L\n";
                return 'l';
            } // L

            else { // Unknown
                return '#';
            }
                        
            break;

        case 4: // I1 FE, F1 FC, M1 FE
        // K U V
        //std::cout << "Case 4\n";  
            if (baseTreeD4_rotationMatrix[3][0] >= -0.200 && baseTreeD4_rotationMatrix[3][0] <= 0.000) { // I1 Spread out
                //std::cout << "I1 FE, F1 FC, M1 FE, K V\n";
                return baseTreeD5(baseTreeD4_rotationMatrix, 3);
            } // K V

            else if (baseTreeD4_rotationMatrix[3][0] >= 0.025 && baseTreeD4_rotationMatrix[3][0] <= 0.175) { // I1 No Spread
                //std::cout << "I1 FE, F1 FC, M1 FE, U\n";
                return 'u';
            } // U

            else if (baseTreeD4_rotationMatrix[3][0] >= 0.175 && baseTreeD4_rotationMatrix[3][0] <= 0.300) { // I1 spread in
                //std::cout << "I1 FE, R1 FE, P1 FC, R\n";
                return 'r';
            } // R

            else { // Unknown
                return '#';
            }
                        
            break;

        case 5: // I1 FC, P3 FC, T1
        // N T
        //std::cout << "Case 5\n";  
        //std::cout << "T2 rot" << baseTreeD4_rotationMatrix[2][0] << '\n';
            if (baseTreeD4_rotationMatrix[2][0] >= 0.000 && baseTreeD4_rotationMatrix[2][0] <= 0.500) { // T2 FE
                //std::cout << "I1 FC, P3 FC, T1, A\n";
                return 'a';
            } // A

            else if (baseTreeD4_rotationMatrix[2][0] >= 0.500 && baseTreeD4_rotationMatrix[2][0] <= 1.610) { // T2 FC
                //std::cout << "I1 FC, P3 FC, T1, S\n";
                return 's';
            } // S

            else { // Unknown
                return '#';
            }            
                        
            break;

        case 6: // I1 FC, P3 FC, T1
        // T N
        //std::cout << "T1 " << baseTreeD4_rotationMatrix[1][0] << '\n';
        std::cout << "Case 6\n";  
            if (baseTreeD4_rotationMatrix[1][0] >= 0.200 && baseTreeD4_rotationMatrix[1][0] <= 0.450) { // I3 FE
                //std::cout << "I1 FC, P3 FC, T1, T\n";
                return 't';
            } // T

            else if (baseTreeD4_rotationMatrix[1][0] >= 0.500 && baseTreeD4_rotationMatrix[1][0] <= 1.000) { // I3 FC
                //std::cout << "I1 FC, P3 FC, T1, N\n";
                return 'n';
            } // N
            else { // Unknown
                return '#';
            }
                        
            break;

// Fix case 5 6 overlap

        case 7: // I1 FC, P3 FE, P1 FE
        // I Y
        // Need custom thumb
        //std::cout << "Case 7\n";  
            if (baseTreeD4_rotationMatrix[1][0] >= 0.000 && baseTreeD4_rotationMatrix[1][0] <= 0.300) {
                //std::cout << "I1 FC, P3 FE, P1 FE, Y\n";
                return 'i';
            } // I

            else if (baseTreeD4_rotationMatrix[1][0] >= -0.700 && baseTreeD4_rotationMatrix[1][0] <= -0.400) {
                //std::cout << "I1 FC, P3 FE, P1 FE, Y\n";
                return 'y';
            } // Y

            else { // Unknown
                return '#';
            }
                        
            break;
        default:
            return '#';
            break;            
    }

}

char baseTreeD3(const std::vector<std::vector<float>>& baseTreeD3_rotationMatrix3, int combinedResult) { // combinedResult up to 8
    // Used an int to continue using switch, much nicer than ifs

    // std::cout
    // << " T1 X: " << baseTreeD3_rotationMatrix3[1][0]
    // << " T1 Z: " << baseTreeD3_rotationMatrix3[1][2]
    // << "\n"; // Debug stuff 

    switch (combinedResult){
        case 1: // I1 FE, R1 FE
            // B E F R W
            if (classifyPinky1(baseTreeD3_rotationMatrix3[12][2]) == CurlState::FULL_EXTEND) { // P1 FE
                //std::cout << "I1 FE, R1 FE, P1 FE\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 1); 
                // The numbers are like the box we moving to, will upload pic
            } // B E F

            else if (classifyPinky1(baseTreeD3_rotationMatrix3[12][2]) == CurlState::FULL_CURL) { // P1 FC
                //std::cout << "I1 FE, R1 FE, P1 FC\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 2); 
            } // R W

            else { // Unknown
                std::cout << "Case 1 fail\n";
                return '@';
            }

            break;

        case 2: // I1 FE, F1 FC
            // K L U V Z
            if (classifyMiddle1(baseTreeD3_rotationMatrix3[6][2]) == CurlState::FULL_CURL &&
                classifyPinky1(baseTreeD3_rotationMatrix3[12][2]) == CurlState::FULL_CURL 
                // Need pinky 1 to not overlap y amdi
        ) { // M1 FC
                //std::cout << "I1 FE, F1 FC, M1 FC\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 3); 
            } // L Z

            else if (classifyMiddle1(baseTreeD3_rotationMatrix3[6][2]) == CurlState::FULL_EXTEND) { // M1 FE
                //std::cout << "I1 FE, F1 FC, M1 FE\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 4); 
            } // K U V

            else { // Unknown
                std::cout << "Case 2 fail\n";                
                return '@';
            }
                    
            break;

        case 3: // I1 FC, P3 FC
            // A N S T
            if (baseTreeD3_rotationMatrix3[1][0] >= -0.250 && baseTreeD3_rotationMatrix3[1][0] <= 0.150 &&
                baseTreeD3_rotationMatrix3[1][2] >= -0.125 && baseTreeD3_rotationMatrix3[1][2] <= 0.075) {
                // T1 "in", needs custom
                //std::cout << "I1 FC, P3 FC, T1 in\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 5); 
            } // N Y

            else if (baseTreeD3_rotationMatrix3[1][0] >= 0.100 && baseTreeD3_rotationMatrix3[1][0] <= 0.750) { 
                // T1 position, needs custom
                //std::cout << "I1 FC, P3 FC, T1 pos\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 6); 
            } // A S

            else { // Unknown
                std::cout << "Case 3 fail\n";            
                return '@';
            } // Some flaws, keep in mind tho
                    
            break;

        case 4: // I1 FC, P3 FE
            // Ik it reuses p1, def a way to combvine to save but idk
            // I M Y
            if (classifyPinky1(baseTreeD3_rotationMatrix3[12][2]) == CurlState::FULL_EXTEND) { // P1 FE
                //std::cout << "I1 FC, P3 FE, P1 FE\n";
                return baseTreeD4(baseTreeD3_rotationMatrix3, 7); 
            } // I Y

            else if (classifyPinky1(baseTreeD3_rotationMatrix3[12][2]) == CurlState::FULL_CURL) { // P1 FC
                //std::cout << "I1 FC, P3 FE, P1 FC\n";
                return 'm';
            } // M

            else { // Unknown
                std::cout << "Case 4 fail\n";                
                return '!';
            }
                        
            break;

        default:
            return '!';
            break;
    }
}

char baseTreeD2(const std::vector<std::vector<float>>& baseTreeD2_rotationMatrix, CurlState result1) { // Each level does a decision process
    switch (result1){
        case CurlState::FULL_EXTEND: // I1 full extended
        //  B E F K L R U V W Z 7 8
            //std::cout << "Juicy full extend\n";
            if (classifyRing1(baseTreeD2_rotationMatrix[9][2]) == CurlState::FULL_EXTEND) { // 
                //std::cout << "Ring extend\n";
                return baseTreeD3(baseTreeD2_rotationMatrix, 1);
            } // B E F R W 7 8

            else if (classifyRing1(baseTreeD2_rotationMatrix[9][2]) == CurlState::FULL_CURL) {
                //std::cout << "Ring crusher\n";
                return baseTreeD3(baseTreeD2_rotationMatrix, 2);
            }

            else { // Unknown
                std::cout << "Fail 1 fail\n";
                return '-';
            }
            
            break;
            
        case CurlState::FULL_CURL: // I1 fully curled
        // A I M N S T Y
            //std::cout << "Curling\n";
            if (classifyPinky3(baseTreeD2_rotationMatrix[14][2]) == CurlState::FULL_CURL) { // P3 FC
                //std::cout << "Yummers pinky curl\n";
                return baseTreeD3(baseTreeD2_rotationMatrix, 3);
            } // A N S T

            else if (classifyPinky3(baseTreeD2_rotationMatrix[14][2]) == CurlState::FULL_EXTEND) { // P3 FE
                //std::cout << "Pinkie's extended\n";
                return baseTreeD3(baseTreeD2_rotationMatrix, 4);
            } // I M Y
                
            else { // Unknown
                std::cout << "Fail 2 fail\n";
                return '-';
            }

            break;
        
        default:
            std::cout << "Detection fail\n";
            return '-';
            break;
    }
}

char baseTreeD1(const std::vector<std::vector<float>>& baseTreeD1_rotationMatrix) { // Maybe should have started at 0 idgaf
    // Def better ways to do this
    if (classifyIndex1(baseTreeD1_rotationMatrix[3][2]) == CurlState::FULL_EXTEND) { // I1 FE
        return baseTreeD2(baseTreeD1_rotationMatrix, CurlState::FULL_EXTEND);
    } //  B E F K L R U V W Z 7 8

    else if (classifyIndex1(baseTreeD1_rotationMatrix[3][2]) == CurlState::FULL_CURL) { // I1 FC
        return baseTreeD2(baseTreeD1_rotationMatrix, CurlState::FULL_CURL);
    } // A I M N S T Y
        
    else { // Unknown
        return '+';
    }

} // Since ω1 is so massive, for readability each depth is getting split into its own function for easy tracking


void decisionTree(const std::vector<std::vector<float>>& decisionTree_rotationMatrix){
    wristState wristEnum = returnWristState(decisionTree_rotationMatrix);
    char temp {};
    //std::cout << "Current X wrist: " << decisionTree_rotationMatrix[0][0] << '\n';
    switch (wristEnum){
        case BASE:
            temp = baseTreeD1(decisionTree_rotationMatrix);
            std::cout << "We detected " << temp << "\n"; // Debug
            break;
        case ROLL: // ω2
            temp = rollTree(decisionTree_rotationMatrix);
            std::cout << "We detected " << temp << "\n"; // Debug
            break;

        case ROLL_AND_PITCH: // ω3
            temp = 'q'; // Only one letter in Q3, q so just directly set it
            std::cout << "We detected " << temp << "\n"; // Debug
            break;

        case YAW:
            temp = yawTree(decisionTree_rotationMatrix);
            std::cout << "We detected " << temp << "\n"; // Debug
            break;

        case REVERSE_ROLL:
            temp = reverseTree(decisionTree_rotationMatrix);
            std::cout << "We detected " << temp << "\n"; // Debug
            break;

        case UNKNOWN:
            std::cout << "Unknown Wrist Rotation\n";
            break;
        default:
            std::cout << "An error has occured in the wrist state";
            break;
    }
} // This is the decision tree function that calls all the other crap


int main() {
    WSADATA wsaData; // Winsock fills this with version and implimntation info, needed by API as seen below
    WSAStartup(MAKEWORD(2, 2), &wsaData); // Initalize the Winsock library

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // Create UDP socket

    sockaddr_in addr{}; // To hold address
    addr.sin_family = AF_INET; // Specifies the address family, in this case IPv4
    addr.sin_port = htons(UDP_PORT); // Sets the port
    addr.sin_addr.s_addr = INADDR_ANY;  // Binds to all local network interfaces

    bind(sock, (sockaddr*)&addr, sizeof(addr)); // Binds socket to the port

    std::vector<float> buffer(NUM_FLOAT); // Buffer for the rotational data

    while (true) {
        frameCounter++;
        std::vector<std::vector<float>> rotations{}; // 2D vector to actually hold the data
        int bytesReceived = recvfrom(sock, (char*)buffer.data(), buffer.size() * sizeof(float), 0, nullptr, nullptr); 
        // Block until package recieved, then copy raw info into buffer
        for (int i{}; i < NUM_FLOAT; i+=3){
            float tempX = buffer[i];
            float tempY = buffer[i+1];
            float tempZ = buffer[i+2];
            // Temp vars to hold rotations
            rotations.push_back({tempX, tempY, tempZ});
            // Add rotations to a more conveniant vector
        }
        // For loop to place everything into a more convenient structure

        if (bytesReceived != NUM_FLOAT * sizeof(float)) {
            printf("Error has occured with data receiving");
        }
        if (frameCounter % 30 == 0){
            //printBoneRotation(rotations, TRACKED_BONE);
            decisionTree(rotations);
        }
    }

    closesocket(sock); // Close the socket
    WSACleanup(); // Un-initialize Winsock
    return 0;
}