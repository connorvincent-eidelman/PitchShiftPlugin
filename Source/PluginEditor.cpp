#include "PluginEditor.h"
#include "PluginProcessor.h"

PitchShiftPluginAudioProcessorEditor::
PitchShiftPluginAudioProcessorEditor(
    PitchShiftPluginAudioProcessor& p)
    : AudioProcessorEditor(&p),
      audioProcessor(p)
{
    setSize(400, 200);

    stereoWidthSlider.setSliderStyle(juce::Slider::Rotary);
    stereoWidthSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    stereoWidthSlider.setPopupDisplayEnabled(true, false, this);
    addAndMakeVisible(stereoWidthSlider);

    stereoWidthAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "stereoWidth", stereoWidthSlider);
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

void PitchShiftPluginAudioProcessorEditor::resized() {
    stereoWidthSlider.setBounds (getLocalBounds().reduced (40));
}
