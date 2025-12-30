#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <complex>
#include <cstring>

juce::AudioProcessorValueTreeState::ParameterLayout PitchShiftPluginAudioProcessor::createParameterLayout()
{
    using namespace juce;

    return {
        std::make_unique<AudioParameterFloat>(
            "stereoWidth",
            "Stereo Width",
            NormalisableRange<float>(0.0f, 2.0f, 0.01f),
            1.0f),

        std::make_unique<AudioParameterFloat>(
            "smear",
            "Smear",
            NormalisableRange<float>(0.0f, 1.0f, 0.01f),
            0.0f)
        ,
        std::make_unique<AudioParameterFloat>(
            "pitch",
            "Pitch Shift",
            NormalisableRange<float>(-12.0f, 12.0f, 0.01f),
            0.0f),
        std::make_unique<AudioParameterFloat>(
            "formant",
            "Formant (semitones)",
            NormalisableRange<float>(-12.0f, 12.0f, 0.01f),
            0.0f)
        ,
        std::make_unique<AudioParameterFloat>(
            "smoothGrains",
            "Smooth Grains",
            NormalisableRange<float>(0.0f, 1.0f, 0.01f),
            1.0f)
    };
}

PitchShiftPluginAudioProcessor::PitchShiftPluginAudioProcessor()
: AudioProcessor(BusesProperties()
        .withInput("Input", juce::AudioChannelSet::stereo(), true)
        .withOutput("Output", juce::AudioChannelSet::stereo(), true)),
    apvts(*this, nullptr, "PARAMS", createParameterLayout())
{
}




PitchShiftPluginAudioProcessor::~PitchShiftPluginAudioProcessor() {}

void PitchShiftPluginAudioProcessor::prepareToPlay(double sampleRate, int /*samplesPerBlock*/)
{
    const int maxSmearMs = 50;
    int bufferSize = (int)(sampleRate * maxSmearMs / 1000.0);

    smearBuffer.setSize(2, bufferSize);
    smearBuffer.clear();

    smearWritePos = 0;
    smearDelayL = 0.0f;
    smearDelayR = 0.0f;
    rng = juce::Random();
    smearTargetL = 0.0f;
    smearTargetR = 0.0f;
    smearTargetCounter = 0;
    // prepare granular grain buffer
    const int maxGrainMs = 50;
    int grainBufferSize = (int)(sampleRate * maxGrainMs / 1000.0);
    grainBuffer.setSize(2, grainBufferSize);
    grainBuffer.clear();

    grainSizeSamples = (int)(0.03 * sampleRate); // default 30 ms grain
    grainPos = 0;
    grainStart = 0;
    grainCounter = 0;
    grainReadPos = 0.0f;

    // prepare FFT formant shifter buffers
    fftSize = (1 << fftOrder);
    fftHop = fftSize / 4;
    fftBufferL.assign(fftSize, 0.0f);
    fftBufferR.assign(fftSize, 0.0f);
    fftWritePos = 0;
    procOutL.clear();
    procOutR.clear();
    // initialize phase-vocoder state arrays for half-spectrum
    int half = fftSize / 2;
    prevPhaseL.assign(half + 1, 0.0f);
    prevPhaseR.assign(half + 1, 0.0f);
    synPhaseL.assign(half + 1, 0.0f);
    synPhaseR.assign(half + 1, 0.0f);
    prevMagL.assign(half + 1, 0.0f);
    prevMagR.assign(half + 1, 0.0f);
    formantInitialized = false;
    // prepare dry delay to match FFT latency
    dryDelayL.assign(fftSize, 0.0f);
    dryDelayR.assign(fftSize, 0.0f);
    dryDelayPos = 0;
    // prepare persistent overlap-add buffers
    olaL.assign(fftSize, 0.0f);
    olaR.assign(fftSize, 0.0f);
    setLatencySamples(fftSize - fftHop);

    // initialize formant smoothing state
    smoothedFormantRatio.reset(sampleRate, 0.05); // 50 ms smoothing
    smoothedFormantRatio.setCurrentAndTargetValue(1.0f);
    lastFormantRatio = 1.0f;
    // initialize FFT priming/crossfade
    fftPrimed = false;
    crossfadeSamplesRemaining = 0;
    crossfadeLength = fftHop; // crossfade over one hop
}

void PitchShiftPluginAudioProcessor::releaseResources() {}

void PitchShiftPluginAudioProcessor::processBlock(
    juce::AudioBuffer<float>& buffer,
    juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    // Simple stereo widener using mid/side processing.
    if (buffer.getNumChannels() >= 2)
    {
        auto* readL = buffer.getReadPointer(0);
        auto* readR = buffer.getReadPointer(1);
        auto* writeL = buffer.getWritePointer(0);
        auto* writeR = buffer.getWritePointer(1);

        // read parameters once per block
        float smearAmount = 0.0f;
        if (auto p = apvts.getRawParameterValue("smear"))
            smearAmount = p->load();

        float width = 1.0f;
        if (auto param = apvts.getRawParameterValue("stereoWidth"))
            width = param->load();

        const int numSamples = buffer.getNumSamples();

        // read pitch/formant/smooth params once per block (avoid per-sample jumps)
        float pitchSemitones = 0.0f;
        if (auto pp = apvts.getRawParameterValue("pitch"))
            pitchSemitones = pp->load();
        float smoothGrains = 1.0f;
        if (auto sp = apvts.getRawParameterValue("smoothGrains"))
            smoothGrains = sp->load();
        // compute target formant ratio from semitones, clamp and smooth it
        float formantSemitones = 0.0f;
        if (auto fp = apvts.getRawParameterValue("formant"))
            formantSemitones = fp->load();
        float targetFormantRatio = std::pow(2.0f, formantSemitones / 12.0f);
        // constrain to a musical, safe range to avoid extreme artifacts
        targetFormantRatio = juce::jlimit(0.5f, 2.0f, targetFormantRatio);
        smoothedFormantRatio.setTargetValue(targetFormantRatio);
        // advance smoothed value once per block and use that for processing
        float formantRatio = smoothedFormantRatio.getNextValue();
        // if formant changed meaningfully, reset phase-vocoder state to avoid discontinuities
        if (std::abs(formantRatio - lastFormantRatio) > 0.001f)
        {
            formantInitialized = false;
        }
        lastFormantRatio = formantRatio;

        // playback rate from pitch in semitones
        float playbackRate = std::pow(2.0f, pitchSemitones / 12.0f);

        // determine hop size influenced by smear, playbackRate and smoothGrains (per-block)
        float baseHop = juce::jmap(smearAmount, 64.0f, (float)(grainSizeSamples * 4));
        int hopSize = juce::jlimit(1, grainSizeSamples * 4, (int)(baseHop * playbackRate * (1.0f - 0.9f * smoothGrains)));

        // pointers to grain buffer
        int gBufferSize = grainBuffer.getNumSamples();
        auto* gbufL = grainBuffer.getWritePointer(0);
        auto* gbufR = grainBuffer.getWritePointer(1);

            for (int i = 0; i < numSamples; ++i)
        {
            const float inL = readL[i];
            const float inR = readR[i];

            float procL = inL;
            float procR = inR;

            // couple smear to formant movements to avoid chaotic interactions
            float effectiveSmear = smearAmount * (1.0f - juce::jmin(1.0f, std::abs(formantSemitones) / 12.0f));
            if (effectiveSmear > 0.001f && gBufferSize > 0 && grainSizeSamples > 0)
            {
                // write incoming audio into grain buffer
                gbufL[grainPos] = inL;
                gbufR[grainPos] = inR;



                // fractional read position within grain, supports variable playbackRate
                float readPosF = grainReadPos;
                // compute integer indices and frac for interpolation
                int idx0 = (grainStart + (int)readPosF) % gBufferSize;
                int idx1 = idx0 + 1; if (idx1 >= gBufferSize) idx1 = 0;
                float frac = readPosF - (int)readPosF;

                float s0L = gbufL[idx0];
                float s1L = gbufL[idx1];
                float s0R = gbufR[idx0];
                float s1R = gbufR[idx1];

                float grainSampleL = s0L * (1.0f - frac) + s1L * frac;
                float grainSampleR = s0R * (1.0f - frac) + s1R * frac;

                // envelope based on fractional position within grain length
                float envPos = grainReadPos / (float)grainSizeSamples;
                if (envPos < 0.0f) envPos = 0.0f;
                if (envPos > 1.0f) envPos = 1.0f;
                float env = 0.5f - 0.5f * std::cos(juce::MathConstants<float>::twoPi * envPos);

                // blend grain with dry signal
                procL = grainSampleL * env + inL * (1.0f - env);
                procR = grainSampleR * env + inR * (1.0f - env);

                // advance fractional read position by playback rate
                grainReadPos += playbackRate;
                if (grainReadPos >= (float)grainSizeSamples)
                    grainReadPos -= (float)grainSizeSamples;

                // advance grain write pos and update grainStart at hop intervals
                if ((grainPos % hopSize) == 0)
                    grainStart = grainPos;

                grainPos++;
                if (grainPos >= gBufferSize)
                    grainPos = 0;
            }

            // --- Formant shifter: run FFT frames when buffer fills, keep procOut fifo
            // Only run FFT/formant pipeline when formant parameter is non-zero (per-feature bypass)
            if (std::abs(formantSemitones) > 1e-6f)
            {
                // push proc samples into fftBuffer
                fftBufferL[fftWritePos] = procL;
                fftBufferR[fftWritePos] = procR;
                fftWritePos++;

                // when fftBuffer is full, perform FFT-based formant warp and overlap-add the result into procOut FIFOs
                if (fftWritePos >= fftSize)
                {
                    // prepare complex arrays (use std::complex to match FFT Complex type)
                    std::vector<std::complex<float>> dataL(fftSize), dataR(fftSize);

                // apply analysis window and fill complex arrays
                std::vector<float> tempInL(fftSize), tempInR(fftSize);
                for (int n = 0; n < fftSize; ++n)
                {
                    tempInL[n] = fftBufferL[n];
                    tempInR[n] = fftBufferR[n];
                }
                window.multiplyWithWindowingTable(tempInL.data(), (size_t)fftSize);
                window.multiplyWithWindowingTable(tempInR.data(), (size_t)fftSize);
                for (int n = 0; n < fftSize; ++n)
                {
                    dataL[n] = std::complex<float>(tempInL[n], 0.0f);
                    dataR[n] = std::complex<float>(tempInR[n], 0.0f);
                }

                fft.perform(reinterpret_cast<juce::dsp::Complex<float>*>(dataL.data()),
                            reinterpret_cast<juce::dsp::Complex<float>*>(dataL.data()), false);
                fft.perform(reinterpret_cast<juce::dsp::Complex<float>*>(dataR.data()),
                            reinterpret_cast<juce::dsp::Complex<float>*>(dataR.data()), false);

                int half = fftSize / 2;
                // extract mag/phase
                std::vector<float> magL(half+1), magR(half+1), phL(half+1), phR(half+1);
                for (int k = 0; k <= half; ++k)
                {
                    float re = dataL[k].real();
                    float im = dataL[k].imag();
                    magL[k] = std::sqrt(re*re + im*im);
                    phL[k] = std::atan2(im, re);

                    re = dataR[k].real();
                    im = dataR[k].imag();
                    magR[k] = std::sqrt(re*re + im*im);
                    phR[k] = std::atan2(im, re);
                }

                // --- Phase-vocoder: compute instantaneous frequency per analysis bin
                std::vector<float> instFreqL(half+1), instFreqR(half+1);
                const float twoPi = juce::MathConstants<float>::twoPi;
                // initialize phase arrays to avoid large jumps on first active frame
                if (!formantInitialized)
                {
                    for (int k = 0; k <= half; ++k)
                    {
                        prevPhaseL[k] = phL[k];
                        prevPhaseR[k] = phR[k];
                        synPhaseL[k] = phL[k];
                        synPhaseR[k] = phR[k];
                    }
                    formantInitialized = true;
                }
                for (int k = 0; k <= half; ++k)
                {
                    float omega = twoPi * (float)k / (float)fftSize; // rad/sample
                    float delta = phL[k] - prevPhaseL[k] - omega * (float)fftHop;
                    // wrap to [-pi, pi]
                    delta = std::fmod(delta + juce::MathConstants<float>::pi, twoPi) - juce::MathConstants<float>::pi;
                    instFreqL[k] = omega + delta / (float)fftHop;
                    prevPhaseL[k] = phL[k];

                    delta = phR[k] - prevPhaseR[k] - omega * (float)fftHop;
                    delta = std::fmod(delta + juce::MathConstants<float>::pi, twoPi) - juce::MathConstants<float>::pi;
                    instFreqR[k] = omega + delta / (float)fftHop;
                    prevPhaseR[k] = phR[k];
                }

                // --- Peak-based formant shifting with collision resolution and transient bypass
                std::vector<float> newMagL(half+1, 0.0f), newMagR(half+1, 0.0f);
                std::vector<float> magAccumL(half+1, 0.0f), magAccumR(half+1, 0.0f);
                std::vector<float> weightAccumL(half+1, 0.0f), weightAccumR(half+1, 0.0f);
                std::vector<float> instFreqAccumL(half+1, 0.0f), instFreqAccumR(half+1, 0.0f);
                std::vector<float> mappedInstFreqL(half+1, 0.0f), mappedInstFreqR(half+1, 0.0f);

                // compute a simple global threshold and spectral flux for transient detection
                float avgMagL = 0.0f, avgMagR = 0.0f;
                for (int k = 0; k <= half; ++k) { avgMagL += magL[k]; avgMagR += magR[k]; }
                avgMagL /= (float)(half + 1);
                avgMagR /= (float)(half + 1);
                const float peakThreshFactor = 0.08f; // relative to average
                float threshL = avgMagL * peakThreshFactor;
                float threshR = avgMagR * peakThreshFactor;

                float spectralFluxL = 0.0f, spectralFluxR = 0.0f;
                for (int k = 0; k <= half; ++k)
                {
                    spectralFluxL += std::max(0.0f, magL[k] - prevMagL[k]);
                    spectralFluxR += std::max(0.0f, magR[k] - prevMagR[k]);
                }
                float fluxNormL = spectralFluxL / (avgMagL * (float)(half + 1) + 1e-9f);
                float fluxNormR = spectralFluxR / (avgMagR * (float)(half + 1) + 1e-9f);
                const float fluxBypassThreshold = 0.20f; // tuned: bypass if flux ratio is high

                // If transient detected, bypass formant-shifting for this frame (copy mags)
                if (fluxNormL > fluxBypassThreshold || fluxNormR > fluxBypassThreshold)
                {
                    for (int k = 0; k <= half; ++k)
                    {
                        newMagL[k] = magL[k];
                        newMagR[k] = magR[k];
                        mappedInstFreqL[k] = instFreqL[k];
                        mappedInstFreqR[k] = instFreqR[k];
                    }
                }
                else
                {
                    // detect local peaks
                    std::vector<int> peaksL, peaksR;
                    for (int k = 1; k < half; ++k)
                    {
                        if (magL[k] > magL[k-1] && magL[k] >= magL[k+1] && magL[k] > threshL)
                            peaksL.push_back(k);
                        if (magR[k] > magR[k-1] && magR[k] >= magR[k+1] && magR[k] > threshR)
                            peaksR.push_back(k);
                    }

                    // peak half-width in bins (small region around a peak)
                    const int peakHalfWidth = juce::jlimit(1, 8, fftSize / 256);

                    // accumulate weighted magnitudes and instFreqs into destination bins
                    for (int pk : peaksL)
                    {
                        int dstCenter = juce::jlimit(0, half, (int)std::lround(pk * formantRatio));
                        for (int off = -peakHalfWidth; off <= peakHalfWidth; ++off)
                        {
                            int srcIdx = pk + off;
                            int dstIdx = dstCenter + off;
                            if (srcIdx < 0 || srcIdx > half || dstIdx < 0 || dstIdx > half) continue;
                            float weight = 1.0f - (std::abs(off) / (float)(peakHalfWidth + 1));
                            magAccumL[dstIdx] += magL[srcIdx] * weight;
                            weightAccumL[dstIdx] += weight;
                            instFreqAccumL[dstIdx] += instFreqL[srcIdx] * weight;
                        }
                    }

                    for (int pk : peaksR)
                    {
                        int dstCenter = juce::jlimit(0, half, (int)std::lround(pk * formantRatio));
                        for (int off = -peakHalfWidth; off <= peakHalfWidth; ++off)
                        {
                            int srcIdx = pk + off;
                            int dstIdx = dstCenter + off;
                            if (srcIdx < 0 || srcIdx > half || dstIdx < 0 || dstIdx > half) continue;
                            float weight = 1.0f - (std::abs(off) / (float)(peakHalfWidth + 1));
                            magAccumR[dstIdx] += magR[srcIdx] * weight;
                            weightAccumR[dstIdx] += weight;
                            instFreqAccumR[dstIdx] += instFreqR[srcIdx] * weight;
                        }
                    }

                    // finalize new magnitude and mapped instantaneous frequency per bin
                    const float backgroundMix = 0.18f;
                    for (int k = 0; k <= half; ++k)
                    {
                        if (weightAccumL[k] > 0.0f)
                        {
                            newMagL[k] = magAccumL[k] / weightAccumL[k];
                            mappedInstFreqL[k] = instFreqAccumL[k] / weightAccumL[k];
                        }
                        else
                        {
                            newMagL[k] = 0.0f;
                            mappedInstFreqL[k] = instFreqL[k];
                        }
                        newMagL[k] += backgroundMix * magL[k];

                        if (weightAccumR[k] > 0.0f)
                        {
                            newMagR[k] = magAccumR[k] / weightAccumR[k];
                            mappedInstFreqR[k] = instFreqAccumR[k] / weightAccumR[k];
                        }
                        else
                        {
                            newMagR[k] = 0.0f;
                            mappedInstFreqR[k] = instFreqR[k];
                        }
                        newMagR[k] += backgroundMix * magR[k];
                    }

                    // gentle normalization to avoid level jumps
                    float maxOld = 0.0001f, maxNew = 0.0001f;
                    for (int k = 0; k <= half; ++k) { maxOld = std::max(maxOld, magL[k]); maxNew = std::max(maxNew, newMagL[k]); }
                    float normL = maxOld / (maxNew + 1e-9f);
                    for (int k = 0; k <= half; ++k) newMagL[k] *= normL;
                    maxOld = 0.0001f; maxNew = 0.0001f;
                    for (int k = 0; k <= half; ++k) { maxOld = std::max(maxOld, magR[k]); maxNew = std::max(maxNew, newMagR[k]); }
                    float normR = maxOld / (maxNew + 1e-9f);
                    for (int k = 0; k <= half; ++k) newMagR[k] *= normR;
                }

                // Advance synthesis phase ONCE per bin using mapped instantaneous freq
                for (int k = 0; k <= half; ++k)
                {
                    synPhaseL[k] += mappedInstFreqL[k] * (float)fftHop;
                    synPhaseR[k] += mappedInstFreqR[k] * (float)fftHop;
                }

                // rebuild complex spectrum from newMag and synPhase
                std::vector<std::complex<float>> outDataL(fftSize), outDataR(fftSize);
                for (int k = 0; k <= half; ++k)
                {
                    float mL = newMagL[k];
                    float pL = synPhaseL[k];
                    outDataL[k] = std::complex<float>(mL * std::cos(pL), mL * std::sin(pL));

                    float mR = newMagR[k];
                    float pR = synPhaseR[k];
                    outDataR[k] = std::complex<float>(mR * std::cos(pR), mR * std::sin(pR));

                    if (k > 0 && k < half)
                    {
                        int nk = fftSize - k;
                        outDataL[nk] = std::conj(outDataL[k]);
                        outDataR[nk] = std::conj(outDataR[k]);
                    }
                }

                // inverse FFT
                fft.perform(reinterpret_cast<juce::dsp::Complex<float>*>(outDataL.data()),
                            reinterpret_cast<juce::dsp::Complex<float>*>(outDataL.data()), true);
                fft.perform(reinterpret_cast<juce::dsp::Complex<float>*>(outDataR.data()),
                            reinterpret_cast<juce::dsp::Complex<float>*>(outDataR.data()), true);

                // inverse FFT produced complex data in outData*, extract real part and apply synthesis window
                std::vector<float> tempOutL(fftSize), tempOutR(fftSize);
                for (int n = 0; n < fftSize; ++n)
                {
                    tempOutL[n] = outDataL[n].real() / (float)fftSize;
                    tempOutR[n] = outDataR[n].real() / (float)fftSize;
                }
                window.multiplyWithWindowingTable(tempOutL.data(), (size_t)fftSize);
                window.multiplyWithWindowingTable(tempOutR.data(), (size_t)fftSize);
                // apply overlap-add compensation gain for Hann with 4x overlap
                const float olaGain = 1.0f / 1.5f; // ~0.6667

                // overlap-add into persistent OLA buffers
                for (int n = 0; n < fftSize; ++n)
                {
                    olaL[n] += tempOutL[n] * olaGain;
                    olaR[n] += tempOutR[n] * olaGain;
                }

                // output only fftHop samples from the OLA buffer
                for (int n = 0; n < fftHop; ++n)
                {
                    procOutL.push_back(olaL[n]);
                    procOutR.push_back(olaR[n]);
                }

                // shift OLA buffers left by hop (use memmove for efficiency)
                int tail = fftSize - fftHop;
                std::memmove(olaL.data(), olaL.data() + fftHop, (size_t)tail * sizeof(float));
                std::memmove(olaR.data(), olaR.data() + fftHop, (size_t)tail * sizeof(float));

                // clear tail
                std::fill(olaL.begin() + tail, olaL.end(), 0.0f);
                std::fill(olaR.begin() + tail, olaR.end(), 0.0f);

                // shift fftBuffer left by hop
                int shift = fftHop;
                for (int n = 0; n < fftSize - shift; ++n)
                {
                    fftBufferL[n] = fftBufferL[n + shift];
                    fftBufferR[n] = fftBufferR[n + shift];
                }
                // zero tail
                for (int n = fftSize - shift; n < fftSize; ++n)
                {
                    fftBufferL[n] = 0.0f;
                    fftBufferR[n] = 0.0f;
                }
                fftWritePos = fftSize - shift;
                // store current mags for next-frame flux detection
                for (int k = 0; k <= half; ++k)
                {
                    prevMagL[k] = magL[k];
                    prevMagR[k] = magR[k];
                }
                }
            }

            // consume processed output if available
            float outFFT_L = 0.0f, outFFT_R = 0.0f;
            if (!procOutL.empty())
            {
                outFFT_L = procOutL.front();
                outFFT_R = procOutR.front();
                procOutL.pop_front();
                procOutR.pop_front();
            }

            // delay dry/path to match FFT latency
            float delayedDryL = dryDelayL[dryDelayPos];
            float delayedDryR = dryDelayR[dryDelayPos];
            // store current proc into delay buffer (so it will be available fftSize samples later)
            dryDelayL[dryDelayPos] = procL;
            dryDelayR[dryDelayPos] = procR;
            dryDelayPos++;
            if (dryDelayPos >= fftSize) dryDelayPos = 0;

            // New policy: always output FFT stream. Pre-fill silence until FFT is primed,
            // then crossfade from delayed dry into FFT output once primed to avoid ducking.
            bool fftPrimedNow = (procOutL.size() >= (size_t)fftHop);
            if (!fftPrimedNow)
            {
                // not yet primed: output silence (pre-fill) to avoid mixing dry bursts
                procL = 0.0f;
                procR = 0.0f;
            }
            else
            {
                // just transitioned to primed -> start crossfade
                if (!fftPrimed && fftPrimedNow)
                {
                    crossfadeSamplesRemaining = crossfadeLength > 0 ? crossfadeLength : fftHop;
                    fftPrimed = true;
                }

                if (crossfadeSamplesRemaining > 0)
                {
                    // alpha grows from 0->1 over crossfadeLength
                    float alpha = 1.0f - (float)crossfadeSamplesRemaining / (float)juce::jmax(1, crossfadeLength);
                    procL = delayedDryL * (1.0f - alpha) + outFFT_L * alpha;
                    procR = delayedDryR * (1.0f - alpha) + outFFT_R * alpha;
                    crossfadeSamplesRemaining--;
                }
                else
                {
                    procL = outFFT_L;
                    procR = outFFT_R;
                }
            }

            // apply stereo width to processed signal (mid/side)
            const float mid = 0.5f * (procL + procR);
            const float side = 0.5f * (procL - procR);
            const float newSide = side * width;
            writeL[i] = mid + newSide;
            writeR[i] = mid - newSide;
        }
    }

}

juce::AudioProcessorEditor*
PitchShiftPluginAudioProcessor::createEditor()
{
    return new PitchShiftPluginAudioProcessorEditor(*this);
}

bool PitchShiftPluginAudioProcessor::hasEditor() const { return true; }

const juce::String PitchShiftPluginAudioProcessor::getName() const
{
    return "Pitch Shift";
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new PitchShiftPluginAudioProcessor();
}

bool PitchShiftPluginAudioProcessor::acceptsMidi() const
{
    return false;
}

bool PitchShiftPluginAudioProcessor::producesMidi() const
{
    return false;
}

bool PitchShiftPluginAudioProcessor::isMidiEffect() const
{
    return false;
}

double PitchShiftPluginAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

//==============================================================================
int PitchShiftPluginAudioProcessor::getNumPrograms()
{
    return 1;
}

int PitchShiftPluginAudioProcessor::getCurrentProgram()
{
    return 0;
}

void PitchShiftPluginAudioProcessor::setCurrentProgram (int)
{
}

const juce::String PitchShiftPluginAudioProcessor::getProgramName (int)
{
    return {};
}

void PitchShiftPluginAudioProcessor::changeProgramName (int, const juce::String&)
{
}

//==============================================================================
void PitchShiftPluginAudioProcessor::getStateInformation (juce::MemoryBlock&)
{
}

void PitchShiftPluginAudioProcessor::setStateInformation (const void*, int)
{
}

bool PitchShiftPluginAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    // Only allow stereo in/out
    if (layouts.getMainInputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    return true;
}
