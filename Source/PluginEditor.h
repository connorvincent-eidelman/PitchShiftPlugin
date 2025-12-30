#pragma once
#include <JuceHeader.h>

class PitchShiftPluginAudioProcessor;

class PitchShiftPluginAudioProcessorEditor
    : public juce::AudioProcessorEditor,
      private juce::Timer
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
    juce::Slider smearSlider;
    juce::Slider pitchSlider;
    juce::Slider formantSlider;
    juce::Slider smoothGrainsSlider;
    juce::Label stereoLabel;
    juce::Label smearLabel;
    juce::Label pitchLabel;
    juce::Label formantLabel;
    juce::Label smoothGrainsLabel;
    using Attachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    std::unique_ptr<Attachment> stereoWidthAttachment;
    std::unique_ptr<Attachment> smearAttachment;
    std::unique_ptr<Attachment> pitchAttachment;
    std::unique_ptr<Attachment> formantAttachment;
    std::unique_ptr<Attachment> smoothGrainsAttachment;
    // on-screen debug HUD
    juce::Label debugLabel;

    void timerCallback() override;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(
        PitchShiftPluginAudioProcessorEditor)
};
