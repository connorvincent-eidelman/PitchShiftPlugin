#include "PluginProcessor.h"
#include "PluginEditor.h"

juce::AudioProcessorValueTreeState::ParameterLayout PitchShiftPluginAudioProcessor::createParameterLayout()
{
    using namespace juce;

    return { std::make_unique<AudioParameterFloat>(
        "stereoWidth",
        "Stereo Width",
        NormalisableRange<float>(0.0f, 2.0f, 0.01f),
        1.0f) };
}

PitchShiftPluginAudioProcessor::PitchShiftPluginAudioProcessor()
: AudioProcessor(BusesProperties()
        .withInput("Input", juce::AudioChannelSet::stereo(), true)
        .withOutput("Output", juce::AudioChannelSet::stereo(), true)),
    apvts(*this, nullptr, "PARAMS", createParameterLayout())
{
}




PitchShiftPluginAudioProcessor::~PitchShiftPluginAudioProcessor() {}

void PitchShiftPluginAudioProcessor::prepareToPlay(double /*sampleRate*/, int /*samplesPerBlock*/) {}

void PitchShiftPluginAudioProcessor::releaseResources() {}

void PitchShiftPluginAudioProcessor::processBlock(
    juce::AudioBuffer<float>& buffer,
    juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    // Simple stereo widener using mid/side processing.
    if (buffer.getNumChannels() >= 2)
    {
        auto* left = buffer.getReadPointer(0);
        auto* right = buffer.getReadPointer(1);
        auto* writeL = buffer.getWritePointer(0);
        auto* writeR = buffer.getWritePointer(1);

        float width = 1.0f;
        if (auto param = apvts.getRawParameterValue("stereoWidth"))
            width = param->load();

        const int numSamples = buffer.getNumSamples();

        for (int i = 0; i < numSamples; ++i)
        {
            const float l = left[i];
            const float r = right[i];

            const float mid = 0.5f * (l + r);
            const float side = 0.5f * (l - r);

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
