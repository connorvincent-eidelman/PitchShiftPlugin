#include "PluginProcessor.h"
#include "PluginEditor.h"

PitchShiftPluginAudioProcessor::PitchShiftPluginAudioProcessor()
: AudioProcessor(BusesProperties()
    .withInput("Input", juce::AudioChannelSet::stereo(), true)
    .withOutput("Output", juce::AudioChannelSet::stereo(), true))
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
