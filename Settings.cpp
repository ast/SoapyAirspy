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

SoapyAirspy::SoapyAirspy(const SoapySDR::Kwargs &args)
{
    deviceId = -1;

    sampleRate = 3000000;
    centerFrequency = 0;

    numBuffers = DEFAULT_NUM_BUFFERS;

    agcMode = false;
    rfBias = false;
    bitPack = false;

    bufferedElems = 0;
    resetBuffer = false;
    
    streamActive = false;
    sampleRateChanged.store(false);
    
    dev = nullptr;
    
    lnaGain = mixerGain = vgaGain = 0;
    
    if (args.count("device_id") != 0)
    {
        try {
            deviceId = std::stoi(args.at("device_id"));
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("device_id invalid.");
        }
        
        std::vector<struct airspy_device *> allDevs;
        
        int status;
        for (int i = 0, iMax = deviceId; i <= iMax; i++) {
            struct airspy_device *searchDev = nullptr;
            status = airspy_open(&searchDev);
    
            if (status != AIRSPY_SUCCESS) {
                continue;
            }
    
            allDevs.push_back(searchDev);
        }

        int numDevices = allDevs.size();

        if (deviceId < 0 || deviceId >= numDevices) {
            for (std::vector< struct airspy_device * >::iterator i = allDevs.begin(); i != allDevs.end(); i++) {
                airspy_close(*i);
            }

            throw std::runtime_error("Airspy device_id out of range [0 .. " + std::to_string(numDevices) + "].");
        }

        dev = allDevs[deviceId];
   
        for (std::vector< struct airspy_device * >::iterator i = allDevs.begin(); i != allDevs.end(); i++) {
            if (*i != dev) {
                airspy_close(*i);
            }
        }
                          
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Found Airspy device using 'device_id' = %d", deviceId);
    }
    
    if (deviceId == -1) {
        throw std::runtime_error("device_id missing.");
    }
}

SoapyAirspy::~SoapyAirspy(void)
{
    airspy_close(dev);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspy::getDriverKey(void) const
{
    return "Airspy";
}

std::string SoapyAirspy::getHardwareKey(void) const
{
    return "Airspy";
}

SoapySDR::Kwargs SoapyAirspy::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["origin"] = "https://github.com/pothosware/SoapyAirspy";
    args["device_id"] = std::to_string(deviceId);

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspy::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    return antennas;
}

void SoapyAirspy::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    // TODO
}

std::string SoapyAirspy::getAntenna(const int direction, const size_t channel) const
{
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspy::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return false;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> results;

    results.push_back("LNA");
    results.push_back("MIX");
    results.push_back("VGA");

    return results;
}

bool SoapyAirspy::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapyAirspy::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    agcMode = automatic;
    
    airspy_set_lna_agc(dev, agcMode?1:0);
    airspy_set_mixer_agc(dev, agcMode?1:0);
    
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AGC: %s", automatic ? "Automatic" : "Manual");
}

bool SoapyAirspy::getGainMode(const int direction, const size_t channel) const
{
    return agcMode;
}

void SoapyAirspy::setGain(const int direction, const size_t channel, const double value)
{
    //set the overall gain by distributing it across available gain elements
    //OR delete this function to use SoapySDR's default gain distribution algorithm...
    SoapySDR::Device::setGain(direction, channel, value);
}

void SoapyAirspy::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    if (name == "LNA")
    {
        lnaGain = uint8_t(value);
        airspy_set_lna_gain(dev, lnaGain);
    }
    else if (name == "MIX")
    {
        mixerGain = uint8_t(value);
        airspy_set_mixer_gain(dev, mixerGain);
    }
    else if (name == "VGA")
    {
        vgaGain = uint8_t(value);
        airspy_set_vga_gain(dev, vgaGain);
    }
}

double SoapyAirspy::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA")
    {
        return lnaGain;
    }
    else if (name == "MIX")
    {
        return mixerGain;
    }
    else if (name == "VGA")
    {
        return vgaGain;
    }

    return 0;
}

SoapySDR::Range SoapyAirspy::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA" || name == "MIX" || name == "VGA") {
        return SoapySDR::Range(0, 15);
    }
 
    return SoapySDR::Range(0, 15);    
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspy::setFrequency(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double frequency,
        const SoapySDR::Kwargs &args)
{
    if (name == "RF")
    {
        centerFrequency = (uint32_t) frequency;
        resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting center freq: %d", centerFrequency);
        airspy_set_freq(dev, centerFrequency);
    }
}

double SoapyAirspy::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "RF")
    {
        return (double) centerFrequency;
    }

    return 0;
}

std::vector<std::string> SoapyAirspy::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyAirspy::getFrequencyRange(
        const int direction,
        const size_t channel,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        results.push_back(SoapySDR::Range(24000000, 1800000000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyAirspy::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspy::setSampleRate(const int direction, const size_t channel, const double rate)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %d", sampleRate);

    if (sampleRate != rate) {
        sampleRate = rate;
        resetBuffer = true;
        sampleRateChanged.store(true);
    }
}

double SoapyAirspy::getSampleRate(const int direction, const size_t channel) const
{
    return sampleRate;
}

std::vector<double> SoapyAirspy::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;

    uint32_t numRates;
	airspy_get_samplerates(dev, &numRates, 0);

	std::vector<uint32_t> samplerates;
    samplerates.resize(numRates);
    
	airspy_get_samplerates(dev, samplerates.data(), numRates);

	for (auto i: samplerates) {
        results.push_back(i);
	}

    return results;
}

void SoapyAirspy::setBandwidth(const int direction, const size_t channel, const double bw)
{
    SoapySDR::Device::setBandwidth(direction, channel, bw);
}

double SoapyAirspy::getBandwidth(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getBandwidth(direction, channel);
}

std::vector<double> SoapyAirspy::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;

    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspy::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;
 
    // Bias-T
    SoapySDR::ArgInfo biasOffsetArg;
    biasOffsetArg.key = "biastee";
    biasOffsetArg.value = "false";
    biasOffsetArg.name = "Bias tee";
    biasOffsetArg.description = "Enable the 4.5v DC Bias tee to power SpyVerter / LNA / etc. via antenna connection.";
    biasOffsetArg.type = SoapySDR::ArgInfo::BOOL;

    // bitpack
    SoapySDR::ArgInfo bitpackOffsetArg;
    bitpackOffsetArg.key = "bitpack";
    bitpackOffsetArg.value = "false";
    bitpackOffsetArg.name = "Bit Pack";
    bitpackOffsetArg.description = "Enable packing 4 12-bit samples into 3 16-bit words for 25% less USB trafic.";
    bitpackOffsetArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(biasOffsetArg);
    setArgs.push_back(bitpackOffsetArg);
 
    return setArgs;
}

void SoapyAirspy::writeSetting(const std::string &key, const std::string &value)
{
    if (key == "biastee") {
        bool enable = (value == "true");

        airspy_set_rf_bias(dev, enable);
    }
	
     if (key == "bitpack") {
        bool enable = (value == "true");

        airspy_set_packing(dev, enable);
    }

}

std::string SoapyAirspy::readSetting(const std::string &key) const
{
    if (key == "biastee") {
        return rfBias?"true":"false";
    }
    if (key == "bitpack") {
        return bitPack?"true":"false";
    }
    
    // SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}
