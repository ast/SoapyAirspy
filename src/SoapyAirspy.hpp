/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Charles J. Cliffe

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once

#include <SoapySDR/ConverterRegistry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.h>
#include <SoapySDR/Types.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <string>
#include <thread>

#include <libairspy/airspy.h>

#include "RingBuffer.hpp"

#define MAX_DEVICES 32
// 65536
#define SOAPY_AIRSPY_STREAM_MTU (1 << 16)

class SoapySDR::Stream {
public:
  virtual ~Stream(void) {}
};

class SoapyAirspy : public SoapySDR::Device {
  enum class Gains { LINEARITY, SENSITIVITY, MANUAL };

  // Usable bandwidth, fraction of the sample rate
  const double BANDWIDTH_FACTOR = 0.75;

  // Device handle
  uint64_t serial_;
  struct airspy_device *dev_;

  // Gains
  // Mixed combinations in low level driver
  uint8_t linearityGain_;
  uint8_t sensitivityGain_;
  // Manual
  uint8_t lnaGain_;
  uint8_t mixerGain_;
  uint8_t vgaGain_;

  // TODO this might not be entirely thread safe.
  // requires some consideration.
  size_t sampleSize_;

  // Gain configuration;
  Gains gains_;

  bool rfBias_;
  bool bitPack_;
  bool agcMode_;

  std::map<double, uint32_t> supportedSampleRates_;
  double sampleRate_;
  double centerFrequency_;
  double currentBandwidth_;

  RingBuffer<uint8_t> ringbuffer_;

  SoapySDR::ConverterRegistry::ConverterFunction converterFunction_;

public:
  // TODO: fix to friend class
  int rx_callback(airspy_transfer *transfer);

  SoapyAirspy(const SoapySDR::Kwargs &args);
  virtual ~SoapyAirspy(void);

  /*******************************************************************
   * Identification API
   ******************************************************************/

  std::string getDriverKey(void) const override;

  std::string getHardwareKey(void) const override;

  SoapySDR::Kwargs getHardwareInfo(void) const override;

  /*******************************************************************
   * Channels API
   ******************************************************************/

  size_t getNumChannels(const int) const override;

  /*******************************************************************
   * Stream API
   ******************************************************************/

  std::vector<std::string>
  getStreamFormats(const int direction, const size_t channel) const override;

  std::string getNativeStreamFormat(const int direction, const size_t channel,
                                    double &fullScale) const override;

  SoapySDR::ArgInfoList getStreamArgsInfo(const int direction,
                                          const size_t channel) const override;

  SoapySDR::Stream *
  setupStream(const int direction, const std::string &format,
              const std::vector<size_t> &channels = std::vector<size_t>(),
              const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

  void closeStream(SoapySDR::Stream *stream) override;

  size_t getStreamMTU(SoapySDR::Stream *stream) const override;

  int activateStream(SoapySDR::Stream *stream, const int flags = 0,
                     const long long timeNs = 0,
                     const size_t numElems = 0) override;

  int deactivateStream(SoapySDR::Stream *stream, const int flags = 0,
                       const long long timeNs = 0) override;

  int readStream(SoapySDR::Stream *stream, void *const *buffs,
                 const size_t numElems, int &flags, long long &timeNs,
                 const long timeoutUs = 100000) override;

  /*******************************************************************
   * Antenna API
   ******************************************************************/

  std::vector<std::string> listAntennas(const int direction,
                                        const size_t channel) const override;

  void setAntenna(const int direction, const size_t channel,
                  const std::string &name) override;

  std::string getAntenna(const int direction,
                         const size_t channel) const override;

  /*******************************************************************
   * Frontend corrections API
   ******************************************************************/

  bool hasDCOffsetMode(const int direction,
                       const size_t channel) const override;

  /*******************************************************************
   * Gain API
   ******************************************************************/

  std::vector<std::string> listGains(const int direction,
                                     const size_t channel) const override;

  bool hasGainMode(const int direction, const size_t channel) const override;

  void setGainMode(const int direction, const size_t channel,
                   const bool automatic) override;

  bool getGainMode(const int direction, const size_t channel) const override;

  void setGain(const int direction, const size_t channel,
               const double value) override;

  void setGain(const int direction, const size_t channel,
               const std::string &name, const double value) override;

  double getGain(const int direction, const size_t channel,
                 const std::string &name) const override;

  SoapySDR::Range getGainRange(const int direction, const size_t channel,
                               const std::string &name) const override;

  /*******************************************************************
   * Frequency API
   ******************************************************************/

  void setFrequency(const int direction, const size_t channel,
                    const std::string &name, const double frequency,
                    const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

  double getFrequency(const int direction, const size_t channel,
                      const std::string &name) const override;

  std::vector<std::string> listFrequencies(const int direction,
                                           const size_t channel) const override;

  SoapySDR::RangeList getFrequencyRange(const int direction,
                                        const size_t channel,
                                        const std::string &name) const override;

  SoapySDR::ArgInfoList
  getFrequencyArgsInfo(const int direction,
                       const size_t channel) const override;

  /*******************************************************************
   * Sample Rate API
   ******************************************************************/

  void setSampleRate(const int direction, const size_t channel,
                     const double rate) override;

  double getSampleRate(const int direction,
                       const size_t channel) const override;

  std::vector<double> listSampleRates(const int direction,
                                      const size_t channel) const override;

  void setBandwidth(const int direction, const size_t channel,
                    const double bw) override;

  double getBandwidth(const int direction, const size_t channel) const override;

  std::vector<double> listBandwidths(const int direction,
                                     const size_t channel) const override;

  /*******************************************************************
   * Utility
   ******************************************************************/

  /*******************************************************************
   * Settings API
   ******************************************************************/

  SoapySDR::ArgInfoList getSettingInfo(void) const override;

  void writeSetting(const std::string &key, const std::string &value) override;

  std::string readSetting(const std::string &key) const override;
};
