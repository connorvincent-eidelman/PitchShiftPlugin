#pragma once
#include <JuceHeader.h>

class PitchShiftPluginAudioProcessor : public juce::AudioProcessor
{
public:
    PitchShiftPluginAudioProcessor();
    ~PitchShiftPluginAudioProcessor() override;

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    const juce::String getName() const override;

private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(PitchShiftPluginAudioProcessor)
};
