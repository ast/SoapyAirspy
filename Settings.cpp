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

#include "SoapyAirspy.hpp"

#include "airspy_fir_kernels.hpp"

SoapyAirspy::SoapyAirspy(const SoapySDR::Kwargs &args)
    : dev_(nullptr), sampleSize_(0), gains_(Gains::LINEARITY), rfBias_(false),
      bitPack_(false), sampleRate_(0), centerFrequency_(0),
      currentBandwidth_(0), ringbuffer_(1 << 22) {

  // TODO: make ringbuffer size configurable
  int ret;

  SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
  SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::SoapyAirspy()");

  for (const auto &arg : args) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::SoapyAirspy(%s) = %s",
                   arg.first.c_str(), arg.second.c_str());
  }

  if (args.count("serial") != 0) {
    try {
      serial_ = std::stoull(args.at("serial"), nullptr, 16);
    } catch (const std::invalid_argument &) {
      throw std::runtime_error("serial is not a hex number");
    } catch (const std::out_of_range &) {
      throw std::runtime_error("serial value of out range");
    }

    // For converting serial to hex string.
    std::stringstream serialstr;

    serialstr << std::hex << serial_;
    // Try to open device.
    ret = airspy_open_sn(&dev_, serial_);

    if (ret != AIRSPY_SUCCESS) {
      // TODO: add ret to error message
      throw std::runtime_error("Unable to open AirSpy device with serial =" +
                               serialstr.str());
    }
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Found AirSpy device: serial = %s",
                   serialstr.str().c_str());
  } else {
    ret = airspy_open(&dev_);
    if (ret != AIRSPY_SUCCESS) {
      throw std::runtime_error("Unable to open AirSpy device");
    }
  }

  // Apply arguments to settings when they match
  for (const auto &info : getSettingInfo()) {
    const auto it = args.find(info.key);
    if (it != args.end()) {
      SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::SoapyAirspy(%s) = %s",
                     info.key.c_str(), it->second.c_str());
      writeSetting(it->first, it->second);
    }
  }

  // Get supported sample rates.
  uint32_t num_rates;
  ret = airspy_get_samplerates(dev_, &num_rates, 0);
  if (ret != AIRSPY_SUCCESS) {
    throw std::runtime_error("Unable to get number of supported sample rates");
  }

  std::vector<uint32_t> supported_rates(num_rates);
  ret = airspy_get_samplerates(dev_, &supported_rates[0], num_rates);
  if (ret != AIRSPY_SUCCESS) {
    throw std::runtime_error("Unable to get supported sample rates");
  }

  // Make sure it's sorted.
  std::sort(supported_rates.begin(), supported_rates.end());

  // Add to map of supported double uint32_t rates.
  for (const auto &rate : supported_rates) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Supported sample rate: %d", rate);
    supportedSampleRates_.insert(std::make_pair(rate, rate));
  }

  // Default to lowest gain, probably the safest.
  setGain(SOAPY_SDR_RX, 0, 0);
  setGainMode(SOAPY_SDR_RX, 0, false);
}

SoapyAirspy::~SoapyAirspy(void) {
  if (dev_ != nullptr) {
    airspy_close(dev_);
    dev_ = nullptr;
  }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspy::getDriverKey(void) const { return "Airspy"; }

std::string SoapyAirspy::getHardwareKey(void) const { return "Airspy"; }

SoapySDR::Kwargs SoapyAirspy::getHardwareInfo(void) const {
  // key/value pairs for any useful information
  // this also gets printed in --probe
  SoapySDR::Kwargs args;

  std::stringstream serialstr;
  serialstr.str("");
  serialstr << std::hex << serial_;
  args["serial"] = serialstr.str();

  return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspy::getNumChannels(const int direction) const {
  return (direction == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listAntennas(const int direction,
                                                   const size_t channel) const {
  std::vector<std::string> antennas;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::listAntennas(%d, %d)",
                 direction, channel);

  if (direction == SOAPY_SDR_RX and channel == 0) {
    antennas.push_back("RX");
  }

  return antennas;
}

void SoapyAirspy::setAntenna(const int direction, const size_t channel,
                             const std::string &name) {
  SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::setAntenna(%d, %d, %s)",
                 direction, channel, name.c_str());
}

std::string SoapyAirspy::getAntenna(const int direction,
                                    const size_t channel) const {

  SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::getAntenna(%d, %d)", direction,
                 channel);

  return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspy::hasDCOffsetMode(const int direction,
                                  const size_t channel) const {

  SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::hasDCOffsetMode(%d, %d)",
                 direction, channel);

  return false;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listGains(const int direction,
                                                const size_t channel) const {
  std::vector<std::string> results;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "SoapyAirspy::listGains(%d, %d) invalid channel", direction,
                   channel);
    return results;
  }

  switch (gains_) {
  case Gains::LINEARITY:
    results.push_back("LIN");
    break;
  case Gains::SENSITIVITY:
    results.push_back("SENS");
    break;
  case Gains::MANUAL:
    results.push_back("LNA");
    results.push_back("MIX");
    results.push_back("VGA");
    break;
  }

  return results;
}

bool SoapyAirspy::hasGainMode(const int direction, const size_t channel) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "SoapyAirspy::hasGainMode(%d, %d) invalid channel",
                   direction, channel);
    return false;
  }

  // This means has AGC.
  return true;
}

void SoapyAirspy::setGainMode(const int direction, const size_t channel,
                              const bool automatic) {
  int ret;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "setGainMode: invalid direction or channel");
    return;
  }

  // LNA
  ret = airspy_set_lna_agc(dev_, automatic);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_lna_agc() failed: %d", ret);
    return;
  }

  // Mixer
  ret = airspy_set_mixer_agc(dev_, automatic);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_mixer_agc() failed: %d", ret);
    return;
  }

  // Store
  agcMode_ = automatic;
}

bool SoapyAirspy::getGainMode(const int direction, const size_t channel) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getGainMode: invalid direction or channel");
    return agcMode_;
  }

  return agcMode_;
}

void SoapyAirspy::setGain(const int direction, const size_t channel,
                          const double value) {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setGain: invalid direction or channel");
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyAirspy::setGain(%d, %d, %f)", direction,
                 channel, value);

  // Set the overall gain by distributing it across available gain
  // elements using the default implementation.
  SoapySDR::Device::setGain(direction, channel, value);
}

void SoapyAirspy::setGain(const int direction, const size_t channel,
                          const std::string &name, const double value) {

  int ret;
  uint8_t gain = static_cast<uint8_t>(std::round(value));

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setGain: invalid direction or channel");
    return;
  }

  // Set the gain element to the specified value.
  if (name == "LIN") {
    ret = airspy_set_linearity_gain(dev_, gain);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_linearity_gain() failed: %d",
                     ret);
      return;
    }
    linearityGain_ = gain;
  } else if (name == "SENS") {
    ret = airspy_set_sensitivity_gain(dev_, gain);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR,
                     "airspy_set_sensitivity_gain() failed: %d", ret);
      return;
    }
    sensitivityGain_ = gain;
  } else if (name == "LNA") {
    ret = airspy_set_lna_gain(dev_, gain);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_lna_gain() failed: %d", ret);
      return;
    }
    // Success
    lnaGain_ = gain;
  } else if (name == "MIX") {
    ret = airspy_set_mixer_gain(dev_, gain);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_mixer_gain() failed: %d",
                     ret);
      return;
    }
    // Success
    mixerGain_ = gain;
  } else if (name == "VGA") {
    ret = airspy_set_vga_gain(dev_, gain);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_vga_gain() failed: %d", ret);
      return;
    }
    // Success
    vgaGain_ = gain;
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setGain(%s) failed: unknown gain",
                   name.c_str());
    return;
  }
}

double SoapyAirspy::getGain(const int direction, const size_t channel,
                            const std::string &name) const {

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getGain: invalid direction or channel");
    return 0.0;
  }

  if (name == "LIN") {
    return linearityGain_;
  } else if (name == "SENS") {
    return sensitivityGain_;
  } else if (name == "LNA") {
    return lnaGain_;
  } else if (name == "MIX") {
    return mixerGain_;
  } else if (name == "VGA") {
    return vgaGain_;
  } else {
    SoapySDR_logf(SOAPY_SDR_ERROR, "getGain() unknown gain name: %s",
                  name.c_str());
    return 0;
  }
}

SoapySDR::Range SoapyAirspy::getGainRange(const int direction,
                                          const size_t channel,
                                          const std::string &name) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getGainRange: invalid direction or channel");
    return SoapySDR::Range(0.0, 0.0);
  }

  if (name == "LIN" or name == "SENS") {
    return SoapySDR::Range(0, 21);
  } else if (name == "LNA" or name == "MIX" or name == "VGA") {
    return SoapySDR::Range(0, 15);
  } else {
    SoapySDR_logf(SOAPY_SDR_ERROR, "getGainRange() unknown gain name: %s",
                  name.c_str());
    return SoapySDR::Range(0, 0);
  }
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspy::setFrequency(const int direction, const size_t channel,
                               const std::string &name, const double frequency,
                               const SoapySDR::Kwargs &args) {

  for (const auto &arg : args) {
    // What are these for?
    SoapySDR_logf(SOAPY_SDR_DEBUG, "%s = %s", arg.first.c_str(),
                  arg.second.c_str());
  }

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR,
                  "setFrequency: invalid direction or channel");
    return;
  }

  if (name == "RF") {
    uint32_t centerFrequency = static_cast<uint32_t>(frequency);
    int ret = airspy_set_freq(dev_, centerFrequency);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_freq() failed: %d", ret);
      return;
    }
    // Success
    centerFrequency_ = frequency;
  }
}

double SoapyAirspy::getFrequency(const int direction, const size_t channel,
                                 const std::string &name) const {
  if (direction != SOAPY_SDR_RX or channel != 0 or name != "RF") {
    SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequency() direction must be RX");
    return 0;
  }

  return centerFrequency_;
}

std::vector<std::string>
SoapyAirspy::listFrequencies(const int direction, const size_t channel) const {
  std::vector<std::string> names;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR,
                  "listFrequencies: invalid direction or channel");
    return names;
  }

  names.push_back("RF");

  return names;
}

SoapySDR::RangeList
SoapyAirspy::getFrequencyRange(const int direction, const size_t channel,
                               const std::string &name) const {
  SoapySDR::RangeList results;

  if (direction != SOAPY_SDR_RX or channel != 0 or name != "RF") {
    SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequencyRange() direction must be RX");
    return results;
  }

  results.push_back(SoapySDR::Range(24000000, 1800000000));

  return results;
}

SoapySDR::ArgInfoList
SoapyAirspy::getFrequencyArgsInfo(const int direction,
                                  const size_t channel) const {
  SoapySDR::ArgInfoList freqArgs;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR,
                  "getFrequencyArgsInfo: invalid direction or channel");
    return freqArgs;
  }

  // TODO: frequency arguments

  return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspy::setSampleRate(const int direction, const size_t channel,
                                const double rate) {
  int ret;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "setSampleRate: invalid direction or channel");
  }

  // Check that it's a supported rate.
  const auto &kv = supportedSampleRates_.find(rate);
  if (kv == supportedSampleRates_.end()) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setSampleRate: unsupported rate: %f",
                   rate);
    return;
  }

  const auto sampleRate = kv->second;
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setSampleRate: %d", sampleRate);

  ret = airspy_set_samplerate(dev_, sampleRate);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_samplerate() failed: %d", ret);
    return;
  }

  sampleRate_ = sampleRate;
}

double SoapyAirspy::getSampleRate(const int direction,
                                  const size_t channel) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getSampleRate() direction must be RX");
    return 0;
  }

  return sampleRate_;
}

std::vector<double> SoapyAirspy::listSampleRates(const int direction,
                                                 const size_t channel) const {
  std::vector<double> results;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "listSampleRates() direction must be RX");
  }

  for (const auto &kv : supportedSampleRates_) {
    results.push_back(kv.first);
  }

  return results;
}

void SoapyAirspy::setBandwidth(const int direction, const size_t channel,
                               const double bandwidth) {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "setBandwidth() invalid direction or channel");
  }

  // Bandwidth 0 means automatic.
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: %f", bandwidth);

  if (bandwidth == 0) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: automatic");
  }

  // Calculate decimation
  const auto decim = sampleRate_ / (bandwidth == 0 ? sampleRate_ : bandwidth);
  uint32_t kernel_size = 0;
  const float *kernel = nullptr;
  int ret = 0;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: %f %f %f", sampleRate_,
                 bandwidth, decim);

  if (decim < 4) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: decimation < 4");

    kernel = KERNEL_2_80;
    kernel_size = KERNEL_2_80_LEN;

  } else if (decim < 8) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: decimation < 8");

    kernel = KERNEL_4_90;
    kernel_size = KERNEL_4_90_LEN;

  } else if (decim < 16) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: decimation < 16");

    kernel = KERNEL_8_100;
    kernel_size = KERNEL_8_100_LEN;

  } else {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth: decimation >= 16");

    kernel = KERNEL_16_110;
    kernel_size = KERNEL_16_110_LEN;
  }

  ret = airspy_set_conversion_filter_float32(dev_, kernel, kernel_size);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "airspy_set_conversion_filter_float32() failed: %d", ret);
    return;
  } else {
    SoapySDR::logf(SOAPY_SDR_DEBUG,
                   "airspy_set_conversion_filter_float32() success");
  }

  currentBandwidth_ = bandwidth;
}

double SoapyAirspy::getBandwidth(const int direction,
                                 const size_t channel) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getBandwidth() invalid direction or channel");
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG, "getBandwidth: %f", currentBandwidth_);

  return currentBandwidth_;
}

std::vector<double> SoapyAirspy::listBandwidths(const int direction,
                                                const size_t channel) const {
  std::vector<double> results;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "listBandwidths(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR,
                  "listBandwidths() invalid direction or channel");
  }

  for (const auto &kv : supportedSampleRates_) {
    results.push_back(BANDWIDTH_FACTOR * kv.first);
  }

  return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspy::getSettingInfo(void) const {
  SoapySDR::ArgInfoList setArgs;

  // Bias-T
  SoapySDR::ArgInfo biasOffsetArg;
  biasOffsetArg.key = "biastee";
  biasOffsetArg.value = "false";
  biasOffsetArg.name = "Bias tee";
  biasOffsetArg.description = "Enable the 4.5v DC Bias tee to power SpyVerter "
                              "/ LNA / etc. via antenna connection.";
  biasOffsetArg.type = SoapySDR::ArgInfo::BOOL;

  setArgs.push_back(biasOffsetArg);

  // bitpack
  SoapySDR::ArgInfo bitPackingArg;
  bitPackingArg.key = "bitpack";
  bitPackingArg.value = "false";
  bitPackingArg.name = "Bit pack";
  bitPackingArg.description = "Enable packing 4 12-bit samples into 3 16-bit "
                              "words for 25% less USB traffic.";
  bitPackingArg.type = SoapySDR::ArgInfo::BOOL;

  setArgs.push_back(bitPackingArg);

  // bitpack
  SoapySDR::ArgInfo gainsArg;
  gainsArg.key = "gains";
  gainsArg.value = "linearity";
  gainsArg.name = "Gain Mode";
  gainsArg.description = "linearity | sensitivity | manual";
  gainsArg.type = SoapySDR::ArgInfo::STRING;

  setArgs.push_back(gainsArg);

  return setArgs;
}

void SoapyAirspy::writeSetting(const std::string &key,
                               const std::string &value) {
  int ret;

  if (key == "biastee") {
    bool enable = (value == "true");
    rfBias_ = enable;

    ret = airspy_set_rf_bias(dev_, enable);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_rf_bias() failed: %d", ret);
    } else {
      SoapySDR::logf(SOAPY_SDR_DEBUG, "airspy_set_rf_bias(%d)", enable);
    }
  } else if (key == "bitpack") {
    bool enable = (value == "true");
    bitPack_ = enable;

    ret = airspy_set_packing(dev_, enable);
    if (ret != AIRSPY_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_packing() failed: %d", ret);
    } else {
      SoapySDR::logf(SOAPY_SDR_DEBUG, "airspy_set_packing(%d)", enable);
    }
  } else if (key == "gains") {
    if (value == "linearity") {
      gains_ = Gains::LINEARITY;
    } else if (value == "sensitivity") {
      gains_ = Gains::SENSITIVITY;
    } else if (value == "manual") {
      gains_ = Gains::MANUAL;
    } else {
      SoapySDR::logf(SOAPY_SDR_ERROR, "Invalid gain mode specified.");
      gains_ = Gains::LINEARITY;
    }
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "writeSetting() unknown key: %s",
                   key.c_str());
  }
}

std::string SoapyAirspy::readSetting(const std::string &key) const {
  if (key == "biastee") {
    return rfBias_ ? "true" : "false";
  } else if (key == "bitpack") {
    return bitPack_ ? "true" : "false";
  } else if (key == "gains") {
    switch (gains_) {
    case Gains::LINEARITY: {
      return "linearity";
    }
    case Gains::SENSITIVITY: {
      return "sensitivity";
    }
    case Gains::MANUAL: {
      return "manual";
    }
    }
  }

  SoapySDR::logf(SOAPY_SDR_ERROR, "readSetting() unknown key: %s", key.c_str());
  return "";
}
