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
