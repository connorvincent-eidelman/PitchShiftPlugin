#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <complex>

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
            "Formant",
            NormalisableRange<float>(0.5f, 2.0f, 0.01f),
            1.0f)
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
    // prepare dry delay to match FFT latency
    dryDelayL.assign(fftSize, 0.0f);
    dryDelayR.assign(fftSize, 0.0f);
    dryDelayPos = 0;
    setLatencySamples(fftSize);
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
        float formantRatio = 1.0f;
        if (auto fp = apvts.getRawParameterValue("formant"))
            formantRatio = fp->load();

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

            if (smearAmount > 0.001f && gBufferSize > 0 && grainSizeSamples > 0)
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
            float formantRatio = 1.0f;
            if (auto fp = apvts.getRawParameterValue("formant"))
                formantRatio = fp->load();

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

                // warp magnitudes by formantRatio (scale frequency axis)
                std::vector<float> newMagL(half+1), newMagR(half+1);
                for (int k = 0; k <= half; ++k)
                {
                    float src = (float)k / formantRatio;
                    if (src <= 0.0f)
                    {
                        newMagL[k] = magL[0];
                        newMagR[k] = magR[0];
                    }
                    else if (src >= half)
                    {
                        newMagL[k] = magL[half];
                        newMagR[k] = magR[half];
                    }
                    else
                    {
                        int i0 = (int)std::floor(src);
                        int i1 = i0 + 1;
                        float frac = src - (float)i0;
                        newMagL[k] = magL[i0] * (1.0f - frac) + magL[i1] * frac;
                        newMagR[k] = magR[i0] * (1.0f - frac) + magR[i1] * frac;
                    }
                }

                // rebuild complex spectrum with original phases but warped magnitudes
                std::vector<std::complex<float>> outDataL(fftSize), outDataR(fftSize);
                for (int k = 0; k <= half; ++k)
                {
                    float mL = newMagL[k];
                    float pL = phL[k];
                    outDataL[k] = std::complex<float>(mL * std::cos(pL), mL * std::sin(pL));

                    float mR = newMagR[k];
                    float pR = phR[k];
                    outDataR[k] = std::complex<float>(mR * std::cos(pR), mR * std::sin(pR));
                    if (k > 0 && k < half)
                    {
                        // mirror negative frequencies using complex conjugate
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
                for (int n = 0; n < fftSize; ++n)
                {
                    procOutL.push_back(tempOutL[n] * olaGain);
                    procOutR.push_back(tempOutR[n] * olaGain);
                }

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
            }

            // consume processed output if available; otherwise use delayed dry to avoid switching to immediate sample
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

            if (!procOutL.empty() || !procOutR.empty())
            {
                // if there is FFT output, use it
                procL = outFFT_L;
                procR = outFFT_R;
            }
            else
            {
                // fallback to delayed dry (aligned) instead of immediate proc
                procL = delayedDryL;
                procR = delayedDryR;
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
