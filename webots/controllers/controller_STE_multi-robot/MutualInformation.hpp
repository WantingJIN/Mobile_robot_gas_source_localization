#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

double calculateEntropy(const std::vector<int>& data) {
    std::map<int, int> frequency;
    for (int value : data) {
        frequency[value]++;
    }

    double entropy = 0.0;
    int total = data.size();
    for (const auto& [value, count] : frequency) {
        double probability = static_cast<double>(count) / total;
        entropy -= probability * std::log2(probability);
    }

    return entropy;
}

double calculateJointEntropy(const std::vector<int>& X, const std::vector<int>& Y) {
    std::map<std::pair<int, int>, int> jointFrequency;
    int n = X.size();

    for (int i = 0; i < n; ++i) {
        jointFrequency[{X[i], Y[i]}]++;
    }

    double jointEntropy = 0.0;
    int total = n;
    for (const auto& [pair, count] : jointFrequency) {
        double probability = static_cast<double>(count) / total;
        jointEntropy -= probability * std::log2(probability);
    }

    return jointEntropy;
}

double calculateMutualInformation(const std::vector<int>& X, const std::vector<int>& Y) {
    if (X.size() != Y.size()) {
        throw std::invalid_argument("Vectors X and Y must have the same size.");
    }

    double entropyX = calculateEntropy(X);
    double entropyY = calculateEntropy(Y);
    double jointEntropy = calculateJointEntropy(X, Y);

    return entropyX + entropyY - jointEntropy;
}

std::vector<int> discretizeData(const std::vector<double>& data, int numBins) {
    double minVal = *std::min_element(data.begin(), data.end());
    double maxVal = *std::max_element(data.begin(), data.end());
    double binSize = (maxVal - minVal) / numBins;
    std::vector<int> discreteData;
    if (abs(maxVal - minVal) < 1e-2 ){
		printf("max and min are the same! set uniform distribution\n");
		// uniform distribution
		for (int i = 0; i < numBins; i++){
			discreteData.push_back(double(1.0)/double(numBins));
		}
		return discreteData;
    }

    for (double value : data) {
        int bin = static_cast<int>((value - minVal) / binSize);
        if (bin == numBins) bin--; // Ensure the max value falls into the last bin
        discreteData.push_back(bin);
    }

    return discreteData;
}
