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

#include <SoapySDR/ConverterRegistry.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <algorithm>
#include <chrono>

#define SOAPY_NATIVE_FORMAT SOAPY_SDR_CS16

std::vector<std::string>
SoapyAirspy::getStreamFormats(const int direction, const size_t channel) const {

  std::vector<std::string> formats;

  if (direction == SOAPY_SDR_RX and channel == 0) {
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
  }

  return formats;
}

std::string SoapyAirspy::getNativeStreamFormat(const int direction,
                                               const size_t channel,
                                               double &fullScale) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getNativeStreamFormat invalid direction or channel");
  }

  fullScale = INT16_MAX;

  return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList
SoapyAirspy::getStreamArgsInfo(const int direction,
                               const size_t channel) const {

  SoapySDR::ArgInfoList streamArgs;

  if (direction != SOAPY_SDR_RX and channel != 0) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "getSteamArgsInfo = %d %d", direction,
                   channel);
  }

  return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

static int rx_callback_(airspy_transfer *transfer) {
  SoapyAirspy *self = (SoapyAirspy *)transfer->ctx;
  return self->rx_callback(transfer);
}

int SoapyAirspy::rx_callback(airspy_transfer *transfer) {
  // TODO: fix constant
  const auto timeout = std::chrono::microseconds(250000);

  const auto to_copy =
      static_cast<size_t>(transfer->sample_count) * sampleSize_;

  const auto copied = ringbuffer_.write_at_least(
      to_copy, timeout,
      [&](uint8_t *begin, [[maybe_unused]] const uint32_t available) {
        // Copy samples to ring buffer.
        std::memcpy(begin, transfer->samples, to_copy);
        // Tell ringbuffer to how many we produced.
        return to_copy;
      });

  if (copied < 0) {
    SoapySDR::logf(SOAPY_SDR_INFO,
                   "SoapyAirspy::rx_callback: ringbuffer write timeout");
    // TODO. Improve overflow handling?
    return 0;
  }

  return 0; // anything else is an error.
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyAirspy::setupStream(const int direction,
                                           const std::string &format,
                                           const std::vector<size_t> &channels,
                                           const SoapySDR::Kwargs &args) {
  int ret;

  if (direction != SOAPY_SDR_RX) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "SoapyAirspy::setupStream(%d, %s, %d, %d) -"
                   " direction must be RX",
                   direction, format.c_str(), channels.size(), args.size());
    return nullptr;
  }

  // Check the channel configuration
  if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
    throw std::runtime_error("setupStream invalid channel selection");
  }

  airspy_sample_type sampleType = AIRSPY_SAMPLE_INT16_IQ;

  // Check the format
  if (format == SOAPY_SDR_CF32) {
    SoapySDR::logf(SOAPY_SDR_INFO, "Using format CF32.");
    sampleType = AIRSPY_SAMPLE_FLOAT32_IQ;
  } else if (format == SOAPY_SDR_CS16) {
    SoapySDR::logf(SOAPY_SDR_INFO, "Using format CS16.");
    sampleType = AIRSPY_SAMPLE_INT16_IQ;
  } else {
    throw std::runtime_error("setupStream invalid format: " + format);
  }

  // Setup our sample size
  sampleSize_ = SoapySDR::formatToSize(format);

  SoapySDR::logf(SOAPY_SDR_DEBUG, "sample type: %d, sample size %d", sampleType,
                 sampleSize_);

  ret = airspy_set_sample_type(dev_, sampleType);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_sample_type() failed: %s (%d)",
                   airspy_error_name((airspy_error)ret), ret);
    return nullptr;
  }

  // TODO: better cast
  return (SoapySDR::Stream *)this;
}

void SoapyAirspy::closeStream(SoapySDR::Stream *stream) {

  int ret = 0;

  if (stream != (SoapySDR::Stream *)this) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "stream != this");
  }

  if (dev_ == nullptr) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "dev_ == nullptr", ret);
    return;
  }

  ret = airspy_stop_rx(dev_);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_stop() = %d", ret);
  }

  ret = airspy_close(dev_);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_close() = %d", ret);
  }
  dev_ = nullptr;

  ringbuffer_.clear();
}

size_t SoapyAirspy::getStreamMTU(SoapySDR::Stream *stream) const {
  if (stream != (SoapySDR::Stream const *)this) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "stream != this");
  }

  return SOAPY_AIRSPY_STREAM_MTU;
}

int SoapyAirspy::activateStream(SoapySDR::Stream *stream, const int flags,
                                const long long timeNs, const size_t numElems) {
  int ret;

  if (stream != (SoapySDR::Stream *)this) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "stream != this");
  }

  if (flags != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "SoapyAirspy::activateStream() - flags not supported");
  }

  SoapySDR::logf(
      SOAPY_SDR_DEBUG,
      "SoapyAirspy::activateStream() flags=%d timeNs=%lld numElems=%d", flags,
      timeNs, numElems);

  // Clear ringbuffer of old samples
  ringbuffer_.clear();

  ret = airspy_start_rx(dev_, rx_callback_, this);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_start_rx() failed: %d", ret);
    return SOAPY_SDR_STREAM_ERROR;
  }

  return 0;
}

int SoapyAirspy::deactivateStream(SoapySDR::Stream *stream, const int flags,
                                  const long long timeNs) {

  int ret = 0;

  if (stream != (SoapySDR::Stream *)this) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "stream != this");
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG,
                 "SoapyAirspy::deactivateStream: flags=%d, timeNs=%lld", flags,
                 timeNs);

  if (flags != 0) {
    SoapySDR::logf(SOAPY_SDR_DEBUG,
                   "SoapyAirspy::deactivateStream() - flags not supported");
  }

  // Stop device
  ret = airspy_stop_rx(dev_);
  if (ret != AIRSPY_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_stop_rx() failed: %d", ret);
    return SOAPY_SDR_STREAM_ERROR;
  }

  return 0;
}

int SoapyAirspy::readStream(SoapySDR::Stream *stream, void *const *buffs,
                            const size_t numElems, int &flags,
                            long long &timeNs, const long timeoutUs) {

  // Some applications require this. We don't use flags.
  flags = 0;

  const auto to_copy =
      std::min(numElems * sampleSize_, getStreamMTU(stream) * sampleSize_);

  const auto copied = ringbuffer_.read_at_least(
      to_copy, std::chrono::microseconds(timeoutUs),
      [&](const uint8_t *begin, [[maybe_unused]] const uint32_t available) {
        // Copy to output buffer
        std::memcpy(buffs[0], begin, to_copy);
        // Consume from ring buffer
        return to_copy;
      });

  if (copied < 0) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "readStream: ringbuffer read timeout %d",
                   ringbuffer_.available());
    return SOAPY_SDR_TIMEOUT;
  }

  // TODO. Fix overflow handling.
  // might need to clear?

  // TODO
  timeNs = 0;
  // timeNs = SoapySDR::ticksToTimeNs( _sampleCount, _sampleRate);

  return static_cast<int>(static_cast<size_t>(copied) / sampleSize_);
}
