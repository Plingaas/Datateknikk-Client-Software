//
// Created by Pling on 23/11/2024.
//

#ifndef CSVWRITER_HPP
#define CSVWRITER_HPP
#include <chrono>
#include <csignal>
#include <thread>
#include <iostream>
#include <fstream>

class CSVWriter {
    std::ofstream file;

public:
    explicit CSVWriter(const std::string& filename, const std::string& header) {
        file.open(filename, std::ios::out | std::ios::app);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        // Write the header if the file is empty
        file.seekp(0, std::ios::end);
        if (file.tellp() == 0) {
            file << header << "\n";
        }
    }

    ~CSVWriter() {
        if (file.is_open()) {
            file.close();
            std::cout << "File safely closed.\n";
        }
    }

    void log(long long secondsSinceEpoch, const std::vector<float>& data) {
        file << secondsSinceEpoch << ",";
        for (int i = 0; i < data.size(); i++) {
            if (i != data.size()-1) {
                file << data[i] << ",";
            } else {
                file << data[i] << "\n";
            }
        }

        file.flush();
    }
};
#endif //CSVWRITER_HPP
