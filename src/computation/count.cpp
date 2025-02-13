#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;

// Function to count points in a .bin file
size_t countPointsInBin(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Unable to open file " << filePath << std::endl;
        return 0;
    }

    // Each point in Kitti dataset is represented as 4 floats (x, y, z, intensity)
    const size_t pointSize = 4 * sizeof(float);

    // Get the file size
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // Calculate the number of points
    size_t numPoints = fileSize / pointSize;
    file.close();

    return numPoints;
}

// Function to write results to a TXT file
void writeResultsToTxt(const std::string& outputFilePath, const std::vector<size_t>& results) {
    std::ofstream outFile(outputFilePath);
    if (!outFile) {
        std::cerr << "Error: Unable to open output file " << outputFilePath << std::endl;
        return;
    }

    // Write data
    for (const auto& pointCount : results) {
        outFile << pointCount << "\n";
    }

    outFile.close();
    std::cout << "Results written to " << outputFilePath << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <input_folder> <output_folder> <output_file_name>\n";
        return EXIT_FAILURE;
    }

    std::string inputFolder = argv[1];
    std::string outputFolder = argv[2];
    std::string outputFile = argv[3];

    // Ensure output folder exists
    fs::create_directories(outputFolder);
    std::cout << "Output folder created: " << outputFolder << std::endl;

    // Vector to store results
    std::vector<size_t> results;

    // Traverse the folder
    for (const auto& entry : fs::directory_iterator(inputFolder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".bin") {
            std::string filePath = entry.path().string();

            // Count points in the current .bin file
            size_t numPoints = countPointsInBin(filePath);
            results.push_back(numPoints);

            std::cout << "Processed " << entry.path().filename().string() << ": " << numPoints << " points" << std::endl;
        }
    }

    // Write results
    std::string outputFilePath = outputFolder + "/" + outputFile +".txt";
    writeResultsToTxt(outputFilePath, results);

    return 0;
}
