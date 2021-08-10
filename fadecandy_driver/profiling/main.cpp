#include <iostream>
#include <vector>
#include <chrono>
#include <cassert>
#include <iomanip>

#include "util.h"

constexpr int NUM_STRIPS = 8;
constexpr int LEDS_PER_STRIP = 64;
constexpr int MAX_INTENSITY = 0xFF;

using namespace std::chrono;

std::vector<std::vector<fadecandy_driver::Color>> createInputs() {
    std::vector<std::vector<fadecandy_driver::Color>> input;

    for (int i = 0; i < NUM_STRIPS; i++) {
        std::vector<fadecandy_driver::Color> strip;

        for (int j = 0; j < LEDS_PER_STRIP; j++) {
            strip.emplace_back(
                (j * MAX_INTENSITY) / LEDS_PER_STRIP,
                (((j + 3) % LEDS_PER_STRIP) * MAX_INTENSITY) / LEDS_PER_STRIP,
                (((j + 7) % LEDS_PER_STRIP) * MAX_INTENSITY) / LEDS_PER_STRIP
            );
        }

        input.emplace_back(strip);
    }

    return input;
}

void dumpStrips(const std::vector<std::vector<fadecandy_driver::Color>>& strips) {
    int numStrips = 0;
    for (auto strip: strips) {
        std::cout << "Strip " << ++numStrips << "\n";
        for (auto led: strip) {
            std::cout << led.r_ << ", " << led.g_ << ", " << led.b_ << "\n";
        }
    }
}

void dumpPackets(const std::vector<std::vector<unsigned char>>& packets) {
    std::cout << std::setfill('0');
    for (auto packet: packets) {
        int i = 0;
        for (auto val: packet) {
            std::cout << std::hex << std::setw(2) << static_cast<int>(val);
            if (++i == 16) {
                i = 0;
                std::cout << "\n";
            } else {
                std::cout << " ";
            }
        }
        std::cout << "\n";
    }
}


void comparePackets(
    const std::vector<std::vector<unsigned char>>& packets1,
    const std::vector<std::vector<unsigned char>>& packets2
) {
    assert(packets1.size() == packets2.size());
    int mismatches = 0;

    for (int i = 0; i < packets1.size(); i++) {
        auto packet1 = packets1[i];
        auto packet2 = packets2[i];
        assert(packet1.size() == packet2.size());

        for (int j = 0; j < packet1.size(); j++) {
            if (packet1[j] != packet2[j]) {
                std::cout << "Mismatch @ " << i << "," << j << "\n";
                ++mismatches;
            }
        }
    }

    assert(mismatches == 0);
}

int main(int, char**) {
    auto v = createInputs();

    comparePackets(
        makeVideoUsbPackets(v),
        makeVideoUsbPacketsImproved(v)
    );

    duration<double, std::milli> time_span1, time_span2;
    {
        auto start = high_resolution_clock::now();
        for (int i = 10000; --i >= 0; ) {
            makeVideoUsbPacketsImproved(v);
        }
        auto end = high_resolution_clock::now();

        time_span1 = end - start;
        std::cout << "Time taken: " << time_span1.count() << " ms\n";
    }

    {
        auto start = high_resolution_clock::now();
        for (int i = 10000; --i >= 0; ) {
            makeVideoUsbPackets(v);
        }
        auto end = high_resolution_clock::now();

        time_span2 = end - start;
        std::cout << "Time taken: " << time_span2.count() << " ms\n";
    }

    std::cout << "Speed up: " << time_span2.count() / time_span1.count() << "\n";
}
