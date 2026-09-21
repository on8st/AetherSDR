// aetherd HL2 Phase 1a — Hl2Spectrum unit test. Feeds synthetic IQ through the
// FFT panadapter path and checks: a complex tone peaks at the expected
// fftshifted bin, DC lands at the centre bin, partial frames accumulate across
// calls, and a large DC offset (the direct-sampling ADC bias) is removed so it
// does not swamp a real tone — mirroring the tools/hl2/spectrum.py behavior.

#include "core/backends/hl2/Hl2Spectrum.h"
#include "core/dsp/WdspChannel.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

using namespace AetherSDR::hl2;

static int g_failures = 0;
static void check(bool cond, const char* what)
{
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    }
}

static constexpr double kPi = 3.14159265358979323846;

// Complex tone at integer bin k0: amp * exp(i 2π k0 n / N).
static std::vector<std::complex<float>> tone(int n, int k0, float amp, std::complex<float> dc = {})
{
    std::vector<std::complex<float>> v(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * k0 * i / n;
        v[static_cast<std::size_t>(i)] =
            amp * std::complex<float>(static_cast<float>(std::cos(ph)),
                                      static_cast<float>(std::sin(ph))) + dc;
    }
    return v;
}

static int argmax(const std::vector<float>& v)
{
    int m = 0;
    for (int i = 1; i < static_cast<int>(v.size()); ++i)
        if (v[static_cast<std::size_t>(i)] > v[static_cast<std::size_t>(m)]) m = i;
    return m;
}

int main()
{
    constexpr int N = 64;
    const int half = N / 2;

    // ---- tone at bin 10 -> peak at fftshifted bin (10 + 32) % 64 = 42 ----
    {
        Hl2Spectrum spec(N);
        std::vector<float> bins;
        const int frames = spec.process(tone(N, 10, 0.5f), bins);
        check(frames == 1, "one frame from N samples");
        check(bins.size() == static_cast<std::size_t>(N), "N bins produced");
        const int peak = argmax(bins);
        check(peak == (10 + half) % N, "tone peaks at expected fftshifted bin");
        check(bins[static_cast<std::size_t>(peak)] > -8.0f, "peak near -6 dBFS (amp 0.5)");
        check(bins[static_cast<std::size_t>((peak + half) % N)] < -30.0f, "opposite bin is floor");
    }

    // ---- DC (bin 0) lands at the centre bin ----
    {
        Hl2Spectrum spec(N);
        std::vector<float> bins;
        // A pure DC tone would be removed by DC subtraction, so use a near-DC
        // bin (k0 = 1) and confirm it maps just off centre, and a real DC-bin
        // signal that survives: feed bin 1.
        spec.process(tone(N, 1, 0.5f), bins);
        check(argmax(bins) == (1 + half) % N, "bin-1 tone maps adjacent to centre");
    }

    // ---- partial frames accumulate across process() calls ----
    {
        Hl2Spectrum spec(N);
        std::vector<float> bins;
        const auto t = tone(N, 7, 0.5f);
        std::span<const std::complex<float>> s(t);
        check(spec.process(s.subspan(0, 30), bins) == 0, "30/64 -> no frame yet");
        check(spec.process(s.subspan(30), bins) == 1, "remaining 34 completes the frame");
        check(argmax(bins) == (7 + half) % N, "accumulated frame decodes the tone");
    }

    // ---- large DC offset is removed, tone survives ----
    {
        Hl2Spectrum spec(N);
        std::vector<float> bins;
        // DC offset of 0.4 on I (as a real HL2 ADC bias) + a smaller tone at bin 20.
        spec.process(tone(N, 20, 0.1f, std::complex<float>(0.4f, 0.0f)), bins);
        const int peak = argmax(bins);
        check(peak == (20 + half) % N, "tone peak survives a large DC offset (DC removed)");
        check(bins[static_cast<std::size_t>(half)] < bins[static_cast<std::size_t>(peak)],
              "centre (DC) bin is below the tone after DC removal");
    }

    // ---- the FFTW planner lock, which merged with nothing covering it ----
    //
    // FFTW's planner is process-global and NOT thread-safe. Hl2Backend::
    // beginDspSetup() constructs an Hl2Spectrum on its worker while
    // WdspChannel::open() plans, allocates and frees through FFTW on another
    // thread -- on EVERY HL2 connect. #5424 serialised it by taking
    // WdspChannel::fftwSetupLock() in this class's constructor and destructor.
    //
    // It landed untested, and says so: the TSan evidence came from
    // radiomodel_pan_id_mapping_test, one of the eight tests deleted as
    // intermittent, and #5443 records the consequence in its own words --
    // "#5424 ... now merges with nothing covering it, because the test that
    // proved it was one of the eight". The race is not intermittent; only the
    // instrument was. This is that coverage, and it needs no sanitizer: hold
    // the lock and observe that construction and destruction WAIT for it.
    //
    // WHAT THIS DOES AND DOES NOT PROVE. It proves the two call sites take the
    // shared lock, which is the fact that can be deleted by an edit. It does
    // not reproduce the race, and no single-threaded assertion could.
    {
        using namespace std::chrono;

        // POSITIVE CONTROL FIRST, so the blocking assertions below cannot pass
        // merely because constructing an Hl2Spectrum is slow. Unlocked, both
        // construction and destruction are microseconds at N=64.
        const auto t0 = steady_clock::now();
        { Hl2Spectrum warm(N); }
        const auto unlockedMs = duration_cast<milliseconds>(steady_clock::now() - t0).count();
        check(unlockedMs < 100,
              "control: an unguarded construct+destroy is far below the wait "
              "window, so a blocked one is the lock and not the work");

        // The window. Generous against a loaded machine, and 2.5x the bound the
        // control above asserts on the unlocked cost.
        constexpr auto kWindow = milliseconds(250);

        // ---- constructor ----
        {
            std::atomic<bool> entered{false};
            std::atomic<bool> constructed{false};
            std::unique_ptr<Hl2Spectrum> spec;

            auto held = WdspChannel::fftwSetupLock();
            std::thread t([&] {
                entered.store(true, std::memory_order_release);
                spec = std::make_unique<Hl2Spectrum>(N);
                constructed.store(true, std::memory_order_release);
            });
            while (!entered.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            std::this_thread::sleep_for(kWindow);
            check(!constructed.load(std::memory_order_acquire),
                  "Hl2Spectrum's constructor BLOCKS while fftwSetupLock() is "
                  "held on another thread (#5424, #5443's defect 1)");
            held.unlock();
            t.join();
            check(constructed.load(std::memory_order_acquire) && spec != nullptr,
                  "and completes as soon as the lock is released");
        }

        // ---- destructor ----
        //
        // Its own assertion rather than a corollary: #5424 guards the frees as
        // well as the plan because the edge TSan named was a free against an
        // allocation, so a patch that kept only the constructor's lock would
        // still leave the reported half open.
        {
            auto spec = std::make_unique<Hl2Spectrum>(N);
            std::atomic<bool> entered{false};
            std::atomic<bool> destroyed{false};

            auto held = WdspChannel::fftwSetupLock();
            std::thread t([&] {
                entered.store(true, std::memory_order_release);
                spec.reset();
                destroyed.store(true, std::memory_order_release);
            });
            while (!entered.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            std::this_thread::sleep_for(kWindow);
            check(!destroyed.load(std::memory_order_acquire),
                  "~Hl2Spectrum BLOCKS on the same lock -- the teardown half "
                  "of the edge, guarded for its own reason");
            held.unlock();
            t.join();
            check(destroyed.load(std::memory_order_acquire) && spec == nullptr,
                  "and completes as soon as the lock is released");
        }
    }

    if (g_failures == 0)
        std::fprintf(stderr, "hl2_spectrum_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
