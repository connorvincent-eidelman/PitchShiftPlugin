#pragma once
#include <JuceHeader.h>
#include <deque>

class PitchShiftPluginAudioProcessor : public juce::AudioProcessor
{
public:
    // debug HUD support (updated from audio thread once-per-block)
    void setDebugText(const juce::String& s);

    PitchShiftPluginAudioProcessor();
    ~PitchShiftPluginAudioProcessor() override;

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    const juce::String getName() const override;

    juce::AudioProcessorValueTreeState apvts;

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    //==============================================================================
    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;


private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(PitchShiftPluginAudioProcessor)
    // Smear buffer for Manipulator-style micro-time diffusion
    juce::AudioBuffer<float> smearBuffer;
    int smearWritePos = 0;
    // Smoothed fractional delay state to avoid per-sample noise
    float smearDelayL = 0.0f;
    float smearDelayR = 0.0f;
    juce::Random rng;
    float smearSmoothFactor = 0.001f; // smoothing for random modulation (slow)
    // target delay and update counter to avoid per-sample random changes
    float smearTargetL = 0.0f;
    float smearTargetR = 0.0f;
    int smearTargetCounter = 0;
    int smearTargetInterval = 1024; // update target every N samples
    // --- Granular smear members (Manipulator-style)
    juce::AudioBuffer<float> grainBuffer;
    int grainSizeSamples = 0;
    int grainPos = 0;
    int grainStart = 0;
    int grainCounter = 0;
    float grainReadPos = 0.0f; // fractional read position for pitch playback
    // --- FFT-based formant shifter members
    int fftOrder = 10; // 2^10 = 1024
    int fftSize = 1 << fftOrder;
    int fftHop = fftSize / 4;
    juce::dsp::FFT fft{ fftOrder };
    juce::dsp::WindowingFunction<float> window{ (size_t)fftSize, juce::dsp::WindowingFunction<float>::hann };
    std::vector<float> fftBufferL;
    std::vector<float> fftBufferR;
    int fftWritePos = 0;
    std::deque<float> procOutL;
    std::deque<float> procOutR;
    // phase-vocoder state (per half-spectrum)
    std::vector<float> prevPhaseL;
    std::vector<float> prevPhaseR;
    std::vector<float> synPhaseL;
    std::vector<float> synPhaseR;
    bool formantInitialized = false;
    // previous-frame magnitudes for transient detection
    std::vector<float> prevMagL;
    std::vector<float> prevMagR;
    // --- Formant smoothing & safety
    float lastFormantRatio = 1.0f;
    juce::SmoothedValue<float> smoothedFormantRatio;
    // dry-path delay to match FFT latency
    std::vector<float> dryDelayL;
    std::vector<float> dryDelayR;
    int dryDelayPos = 0;
    // overlap-add persistent buffer for streaming FFT output
    std::vector<float> olaL;
    std::vector<float> olaR;
    // Debug HUD text (updated from audio thread) and dirty flag
    juce::String debugText;
    std::atomic<bool> debugDirty { false };
    // FFT output priming / crossfade state
    bool fftPrimed = false;
    int crossfadeSamplesRemaining = 0;
    int crossfadeLength = 0;
};
