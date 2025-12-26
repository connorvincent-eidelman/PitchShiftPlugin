#include "PluginProcessor.h"
#include "PluginEditor.h"

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

        // read parameters
        float smearAmount = 0.0f;
        if (auto p = apvts.getRawParameterValue("smear"))
            smearAmount = p->load();

        float width = 1.0f;
        if (auto param = apvts.getRawParameterValue("stereoWidth"))
            width = param->load();

        const int numSamples = buffer.getNumSamples();

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

            // read pitch/formant/smooth params
            float pitchSemitones = 0.0f;
            if (auto pp = apvts.getRawParameterValue("pitch"))
                pitchSemitones = pp->load();
            float smoothGrains = 1.0f;
            if (auto sp = apvts.getRawParameterValue("smoothGrains"))
                smoothGrains = sp->load();

            // playback rate from pitch in semitones
            float playbackRate = std::pow(2.0f, pitchSemitones / 12.0f);

            if (smearAmount > 0.001f && gBufferSize > 0 && grainSizeSamples > 0)
            {
                // write incoming audio into grain buffer
                gbufL[grainPos] = inL;
                gbufR[grainPos] = inR;

                // determine hop size influenced by smear, playbackRate and smoothGrains
                float baseHop = juce::jmap(smearAmount, 64.0f, (float)(grainSizeSamples * 4));
                int hopSize = juce::jlimit(1, grainSizeSamples * 4, (int)(baseHop * playbackRate * (1.0f - 0.9f * smoothGrains)));

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
