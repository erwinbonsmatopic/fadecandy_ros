/*
 * Copyright (c) 2021 Eurotec, Netherlands
 * All rights reserved.
 *
 * Author: Jad Haj Mustafa
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <list>
#include <math.h>
#include <stdexcept>

#include "./util.h"

namespace fadecandy_driver
{
constexpr int LOOKUP_VALUES_PER_PACKET = 31;
constexpr int LOOKUP_VALUES_PER_CHANNEL = 257;
constexpr int USB_PACKET_SIZE = 64;
constexpr int PACKET_TYPE_VIDEO = 0x00;
constexpr int PACKET_TYPE_LUT = 0x40;
constexpr int FINAL_PACKET_BIT = 0x20;
constexpr int LEDS_PER_STRIP = 64;
constexpr int NUM_STRIPS = 8;
constexpr int LEDS_PER_PACKET = (USB_PACKET_SIZE - 1) / 3;

constexpr int NUM_FULL_PACKETS = (NUM_STRIPS * LEDS_PER_STRIP) / LEDS_PER_PACKET;
constexpr int LEDS_IN_LAST_PACKET = (NUM_STRIPS * LEDS_PER_STRIP) % LEDS_PER_PACKET;

Color::Color(int r, int g, int b) : r_(r), g_(g), b_(b)
{
}

std::vector<unsigned char> intToCharArray(int in, const size_t bytes_per_int)
{
  if (in > pow(2, bytes_per_int * 8))
  {
    throw std::overflow_error("Overflow error while converting integer " + std::to_string(in) + " to char array of " +
                              std::to_string(bytes_per_int) + " bytes");
  }
  unsigned char buffer[bytes_per_int];
  std::vector<unsigned char> char_array;
  for (size_t i = 0; i < bytes_per_int; i++)
  {
    size_t shift = 8 * (bytes_per_int - 1 - i);
    buffer[i] = (in >> shift) & 0xff;
    char_array.push_back(buffer[i]);
  }
  std::reverse(char_array.begin(), char_array.end());
  return char_array;
}

std::vector<std::vector<unsigned char>> makeVideoUsbPackets(const std::vector<std::vector<Color>>& led_array_colors)
{
  // Converts 2D array to 1D array, with extra null values
  // Note: This loop and the all_led_colors memory allocation can be avoided.
  int led_index = 0;
  std::vector<Color> all_led_colors(LEDS_PER_STRIP * NUM_STRIPS, { 0, 0, 0 });
  for (size_t i = 0; i < led_array_colors.size(); i++)
  {
    for (size_t j = 0; j < led_array_colors[i].size(); j++)
    {
      led_index = (i * LEDS_PER_STRIP) + j; // Note: led_index++ is faster
      all_led_colors[led_index] = led_array_colors[i][j];
    }
  }
  std::vector<std::vector<unsigned char>> packets;
  std::vector<Color> packet_leds;
  int control;

  while (all_led_colors.size() > 0)
  {
    // Color_bytes is allocated for each iteration of the loop. It's faster to declare this
    // temporary data structure once, outside the loop. Or even better, to not use it at all.
    std::vector<int> color_bytes;
    std::vector<unsigned char> packet;

    if (all_led_colors.size() < LEDS_PER_PACKET)
    {
      // Last packet, copy remaining entries
      packet_leds.assign(all_led_colors.begin(), all_led_colors.end());
      all_led_colors.erase(all_led_colors.begin(), all_led_colors.end());
    }
    else
    {
      // Copy LEDS_PER_PACKET entries
      packet_leds.assign(all_led_colors.begin(), all_led_colors.begin() + LEDS_PER_PACKET);
      // Note: erasing elements from start of vector is a relatively expensive operation and best
      // avoided. Could instead use index/iterator to track which elements have been consumed
      // transferred
      all_led_colors.erase(all_led_colors.begin(), all_led_colors.begin() + LEDS_PER_PACKET);
    }

    control = packets.size() | PACKET_TYPE_VIDEO;
    if (all_led_colors.size() == 0)
    {
      control |= FINAL_PACKET_BIT;
    }

    for (size_t i = 0; i < packet_leds.size(); i++)
    {
      color_bytes.push_back(packet_leds[i].r_);
      color_bytes.push_back(packet_leds[i].g_);
      color_bytes.push_back(packet_leds[i].b_);
    }
    // construnt USB packet and leave the first byte for the control byte
    // Pack packet with zeroes
    if ((USB_PACKET_SIZE - 1) - color_bytes.size() > 0)
    {
      int j = (USB_PACKET_SIZE - 1) - color_bytes.size();
      for (int i = 0; i < j; i++)
      {
        color_bytes.push_back(0);
      }
    }

    // Convert ints to char. However, as long as bytes_per_int is one this relatively expensive
    // loop (with function invocation and two dynamic memory allocations) per iteration can be
    // avoided.
    for (size_t i = 0; i < color_bytes.size(); i++)
    {
      int bytes_per_int = 1;
      std::vector<unsigned char> bufferv;
      bufferv = intToCharArray(color_bytes[i], bytes_per_int);
      packet.insert(packet.end(), bufferv.begin(), bufferv.end());
    }

    // add control byte
    // Note: This is an expensive operation. It needlessly shifts all entries in the array.
    // The control byte should therefore be added before adding the other data.
    packet.insert(packet.begin(), static_cast<unsigned char>(control));

    if (packet.size() != USB_PACKET_SIZE)
    {
      throw std::runtime_error("Packet size does not match the allowed USB packet size");
    }
    packets.push_back(packet);
  }
  return packets;
}


std::vector<std::vector<unsigned char>> makeVideoUsbPacketsImproved(
  const std::vector<std::vector<Color>>& led_array_colors
) {
  std::vector<std::vector<unsigned char>> packets;
  packets.reserve(NUM_FULL_PACKETS + 1);

  int i = 0, j = 0;
  while (packets.size() < NUM_FULL_PACKETS) {
    std::vector<unsigned char> packet(USB_PACKET_SIZE, 0);
    int leds_in_packet = 0;

    char control = static_cast<unsigned char>(packets.size() | PACKET_TYPE_VIDEO);
    packet[0] = control;

    while (leds_in_packet < LEDS_PER_PACKET) {
      // Copy actual colors from current strip
      int num_colors_to_copy_from_strip = std::min(
        LEDS_PER_PACKET - leds_in_packet, static_cast<int>(led_array_colors[i].size()) - j
      );
      int p = leds_in_packet * 3;
      while (--num_colors_to_copy_from_strip >= 0) {
        auto color = led_array_colors[i][j];
        packet[++p] = static_cast<unsigned char>(color.r_);
        packet[++p] = static_cast<unsigned char>(color.g_);
        packet[++p] = static_cast<unsigned char>(color.b_);
        ++leds_in_packet;
        ++j;
      }

      int num_zeroes_to_copy_from_strip = std::min(LEDS_PER_PACKET - leds_in_packet, LEDS_PER_STRIP - j);
      // Nothing needs copying. Packet is pre-populated with zeroes
      j += num_zeroes_to_copy_from_strip;
      leds_in_packet += num_zeroes_to_copy_from_strip;

      if (j == LEDS_PER_STRIP) {
        ++i;
        j = 0;
      }
    }

    packets.push_back(packet);
  }

  // Create last packet
  {
    std::vector<unsigned char> packet(USB_PACKET_SIZE, 0);

    char control = static_cast<unsigned char>(packets.size() | PACKET_TYPE_VIDEO | FINAL_PACKET_BIT);
    packet[0] = control;

    // Copy actual colors from current strip
    int num_colors_to_copy_from_strip = std::min(
      LEDS_PER_PACKET, static_cast<int>(led_array_colors[i].size()) - j
    );
    int p = 0;

    while (--num_colors_to_copy_from_strip >= 0) {
      auto color = led_array_colors[i][j];
      packet[++p] = static_cast<unsigned char>(color.r_);
      packet[++p] = static_cast<unsigned char>(color.g_);
      packet[++p] = static_cast<unsigned char>(color.b_);
      ++j;
    }

    packets.push_back(packet);
  }

  return packets;
}

std::vector<std::vector<unsigned char>> makeLookupTablePackets(const std::vector<int>& red_lookup_values,
                                                               const std::vector<int>& green_lookup_values,
                                                               const std::vector<int>& blue_lookup_values)
{
  if (red_lookup_values.size() != LOOKUP_VALUES_PER_CHANNEL ||
      green_lookup_values.size() != LOOKUP_VALUES_PER_CHANNEL || blue_lookup_values.size() != LOOKUP_VALUES_PER_CHANNEL)
  {
    throw std::runtime_error("Lookup values per channel is not correct");
  }
  std::vector<std::vector<unsigned char>> packets;
  std::vector<int> remaining_lookup_values;
  std::vector<int> packet_lookup_values;
  int control;

  remaining_lookup_values.insert(remaining_lookup_values.begin(), red_lookup_values.begin(), red_lookup_values.end());
  remaining_lookup_values.insert(remaining_lookup_values.end(), green_lookup_values.begin(), green_lookup_values.end());
  remaining_lookup_values.insert(remaining_lookup_values.end(), blue_lookup_values.begin(), blue_lookup_values.end());

  while (remaining_lookup_values.size() > 0)
  {
    std::vector<unsigned char> packet;

    if (remaining_lookup_values.size() < LOOKUP_VALUES_PER_PACKET)
    {
      packet_lookup_values.assign(remaining_lookup_values.begin(), remaining_lookup_values.end());
      remaining_lookup_values.erase(remaining_lookup_values.begin(), remaining_lookup_values.end());
    }
    else
    {
      packet_lookup_values.assign(remaining_lookup_values.begin(),
                                  remaining_lookup_values.begin() + LOOKUP_VALUES_PER_PACKET);
      remaining_lookup_values.erase(remaining_lookup_values.begin(),
                                    remaining_lookup_values.begin() + LOOKUP_VALUES_PER_PACKET);
    }
    control = packets.size() | PACKET_TYPE_LUT;
    if (remaining_lookup_values.size() == 0)
    {
      control |= FINAL_PACKET_BIT;
    }

    if (LOOKUP_VALUES_PER_PACKET - packet_lookup_values.size() > 0)
    {
      int j = LOOKUP_VALUES_PER_PACKET - packet_lookup_values.size();
      for (int i = 0; i < j; i++)
      {
        packet_lookup_values.push_back(0);
      }
    }

    for (size_t i = 0; i < packet_lookup_values.size(); i++)
    {
      int bytes_per_int = 2;
      std::vector<unsigned char> bufferv;
      bufferv = intToCharArray(packet_lookup_values[i], bytes_per_int);
      packet.insert(packet.end(), bufferv.begin(), bufferv.end());
    }
    // add control byte
    packet.insert(packet.begin(), static_cast<unsigned char>(0));
    packet.insert(packet.begin(), static_cast<unsigned char>(control));

    if (packet.size() != USB_PACKET_SIZE)
    {
      throw std::runtime_error("Packet size does not match the allowed USB packet size");
    }
    packets.push_back(packet);
  }
  return packets;
}

std::vector<int> makeDefaultLookupTable()
{
  //
  // color correction curve borrowed from the USB example in the main fadecandy repo:
  //
  // https://github.com/scanlime/fadecandy/blob/master/examples/python/usb-lowlevel.py
  //
  std::vector<int> lookup_values;
  for (int row = 0; row < 257; row++)
  {
    lookup_values.push_back(std::min(0xFFFF, int(pow(row / 256.0, 2.2) * 0x10000)));
  }
  return lookup_values;
}
}  // namespace fadecandy_driver
