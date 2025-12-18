#include "PluginEditor.h"
#include "PluginProcessor.h"

PitchShiftPluginAudioProcessorEditor::
PitchShiftPluginAudioProcessorEditor(
    PitchShiftPluginAudioProcessor& p)
    : AudioProcessorEditor(&p),
      audioProcessor(p)
{
    setSize(400, 200);
}

PitchShiftPluginAudioProcessorEditor::
~PitchShiftPluginAudioProcessorEditor() {}

void PitchShiftPluginAudioProcessorEditor::paint(
    juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);

    g.setColour(juce::Colours::white);
    g.setFont(15.0f);
    g.drawFittedText(
        "Pitch Shift",
        getLocalBounds(),
        juce::Justification::centred,
        1);
}

void PitchShiftPluginAudioProcessorEditor::resized() {}
