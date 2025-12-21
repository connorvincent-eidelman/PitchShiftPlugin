#pragma once
#include <JuceHeader.h>

class PitchShiftPluginAudioProcessor;

class PitchShiftPluginAudioProcessorEditor
    : public juce::AudioProcessorEditor
{
public:
    explicit PitchShiftPluginAudioProcessorEditor(
        PitchShiftPluginAudioProcessor&);
    ~PitchShiftPluginAudioProcessorEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    PitchShiftPluginAudioProcessor& audioProcessor;
    juce::Slider stereoWidthSlider;
    using Attachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    std::unique_ptr<Attachment> stereoWidthAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(
        PitchShiftPluginAudioProcessorEditor)
};
