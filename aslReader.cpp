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
    else if (returnWristState_RotationMatrix[0] >= std::vector<float>{-0.100, -0.175, -0.300} &&  
        returnWristState_RotationMatrix[0] <= std::vector<float>{0.245, 0.115, 0.300}){ // Base state
        //std::cout << "This is in base state\n\n";
        return BASE;
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
    if (rollTree_rotationMatrix[3][2] >= -1.000 && rollTree_rotationMatrix[3][2] <= -0.700) {
        return 'o'; 
    } // Detect O     

    else if (rollTree_rotationMatrix[3][2] >=  -0.250 && rollTree_rotationMatrix[3][2] <= -0.050 && // I1 semi-extend
            rollTree_rotationMatrix[6][2] >= -0.180 && rollTree_rotationMatrix[6][2] <= -0.000) { // M1 semi-extend
        return 'c';
    } // Detects C   

    if (rollTree_rotationMatrix[3][2] >= -0.300 && rollTree_rotationMatrix[3][2] <= -0.100) { // I1 FE (Semi FE for O)
        if (rollTree_rotationMatrix[6][2] >= -0.950 && rollTree_rotationMatrix[6][2] <= -0.300) { // M1 semi-curl 
            return 'd';
        } // Detects D 

        else if (rollTree_rotationMatrix[6][2] >= -1.600 && rollTree_rotationMatrix[6][2] <= -1.100 && // M1 FC
                rollTree_rotationMatrix[4][2] >= -1.600 && rollTree_rotationMatrix[4][2] <= -0.700) { // I2 FC
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

    std::cout
    << " T1 X: " << yawTree_rotationMatrix[1][0]
    << " T1 z: " << yawTree_rotationMatrix[1][2]
    << "\n"; // Debug stuff

    if (yawTree_rotationMatrix[6][2] >= -0.525 && yawTree_rotationMatrix[6][2] <= -0.070) { // M1 FE (H, P)
        if (yawTree_rotationMatrix[7][2] >= -0.550 && yawTree_rotationMatrix[7][2] <= -0.000 && // M2 FE
                yawTree_rotationMatrix[6][0] >= -0.325 && yawTree_rotationMatrix[6][0] <= 0.450) { // I1 Yaw
            return 'h';
        } // H

        else if (yawTree_rotationMatrix[3][2] >= -0.300 && yawTree_rotationMatrix[3][2] <= -0.100) { // I1 FE 
            return 'p'; 
        } // P

        else {
            return '+';
        } // Not H or P
        
    }

    else if (yawTree_rotationMatrix[6][2] >= -1.610 && yawTree_rotationMatrix[6][2] <= -0.450) { // M1 FC (G, J 10)
        if (yawTree_rotationMatrix[12][2] >= -0.700 && yawTree_rotationMatrix[12][2] <= -0.450) { // Checks P1 (Not using P2, not needed)
            return 'j';
        } // J
        
        else if (yawTree_rotationMatrix[3][2] >= -0.500 && yawTree_rotationMatrix[3][2] <= -0.275) { // I1 FE
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


void decisionTree(const std::vector<std::vector<float>>& decisionTree_rotationMatrix){
    wristState wristEnum = returnWristState(decisionTree_rotationMatrix);
    char temp {};
    //std::cout << "Current X wrist: " << decisionTree_rotationMatrix[0][0] << '\n';
    switch (wristEnum){
        case BASE:
            std::cout << "Ello im a base placeholder\n"; // This will call a whole new function, as ω1 is fucking huge
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
            std::cout << "Ello im a reverse placeholder\n";
            break;

        case UNKNOWN:
            std::cout << "Galunga\n";
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